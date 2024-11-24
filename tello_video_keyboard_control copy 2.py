import cv2
from djitellopy import Tello
import keyboard_control as kc
import time
import mediapipe as mp  # Импортируем библиотеку MediaPipe
import numpy as np

# Инициализация модуля управления клавиатурой
kc.init()

# Инициализация дрона
drone = Tello()
drone.connect()
print(f"Уровень заряда батареи: {drone.get_battery()}%")
drone.streamon()

# Загрузка каскадного классификатора для обнаружения лиц
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Инициализация MediaPipe для отслеживания рук
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Инициализация MediaPipe для отслеживания позы
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

# Загрузка YOLO для обнаружения объектов
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")  # Убедитесь, что файлы yolov3.weights и yolov3.cfg находятся в рабочем каталоге
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Флаг для отслеживания состояния полета дрона
is_flying = False

# Функция для обработки команд с клавиатуры
def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kc.getKey("LEFT"): lr = -speed
    elif kc.getKey("RIGHT"): lr = speed

    if kc.getKey("UP"): fb = speed
    elif kc.getKey("DOWN"): fb = -speed

    if kc.getKey("w"): ud = speed
    elif kc.getKey("s"): ud = -speed

    if kc.getKey("a"): yv = -speed
    elif kc.getKey("d"): yv = speed

    if kc.getKey("q"):
        global is_flying
        if not is_flying:
            try:
                drone.takeoff()
                is_flying = True  # Устанавливаем флаг полета
                print("Дрон взлетел!")
            except Exception as e:
                print(f"Ошибка при взлете: {e}")
        else:
            try:
                drone.land()
                is_flying = False  # Сбрасываем флаг полета
                print("Дрон приземлился!")
            except Exception as e:
                print(f"Ошибка при приземлении: {e}")

    return [lr, fb, ud, yv]

# Функция для отслеживания лица
def track_face(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        return (x, y, w, h)  # Возвращаем координаты лица
    return None

# Функция для отслеживания рук
def track_hands(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            for landmark in hand_landmarks.landmark:
                h, w, _ = frame.shape
                cx, cy = int(landmark.x * w), int(landmark.y * h)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)  # Рисуем круги на руках
    return frame

# Функция для отслеживания объектов
def track_objects(frame):
    height, width, _ = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outputs = net.forward(output_layers)

    boxes = []
    confidences = []
    class_ids = []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:  # Порог уверенности
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Прямоугольник вокруг объекта
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Удаление дублирующихся прямоугольников
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Рисуем прямоугольник
            cv2.putText(frame, label, (x, y + 30), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)  # Подписываем объект

    return frame

# Функция для отслеживания тела
def track_body(frame):
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(frame_rgb)
    if results.pose_landmarks:
        mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)  # Рисуем обводку тела
    return frame

# Загрузка классов объектов
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

try:
    while True:
        # Получение команд управления
        vals = getKeyboardInput()
        if is_flying:  # Отправляем команды только если дрон в воздухе
            drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        # Получение видеопотока с камеры дрона
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))

        # Отслеживание лица
        face_position = track_face(frame)
        if face_position:
            x, y, w, h = face_position
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Логика управления дроном для следования за лицом
            center_x = x + w // 2
            if center_x < 320:  # Если лицо слева от центра
                vals[0] = -50  # Двигаться влево
            elif center_x > 320:  # Если лицо справа от центра
                vals[0] = 50  # Двигаться вправо
            else:
                vals[0] = 0  # Остановить движение влево/вправо

        # Отслеживание рук
        frame = track_hands(frame)

        # Отслеживание объектов
        frame = track_objects(frame)

        # Отслеживание тела
        frame = track_body(frame)

        cv2.imshow("Tello Camera", frame)

        # Выход из программы при нажатии клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pass
finally:
    try:
        if drone.is_flying:
            drone.land()  # Приземление дрона
    except Exception as e:
        print(f"Ошибка при приземлении: {e}")
    drone.streamoff()
    cv2.destroyAllWindows()