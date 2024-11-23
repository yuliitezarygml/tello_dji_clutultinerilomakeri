import cv2
from djitellopy import Tello
import keyboard_control as kc
import time

# Инициализация модуля управления клавиатурой
kc.init()

# Инициализация дрона
drone = Tello()
drone.connect()
print(f"Уровень заряда батареи: {drone.get_battery()}%")
drone.streamon()

# Загрузка каскадного классификатора для обнаружения лиц
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

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
        if not drone.is_flying:
            try:
                drone.takeoff()
                print("Дрон взлетел!")
            except Exception as e:
                print(f"Ошибка при взлете: {e}")
        else:
            try:
                drone.land()
                print("Дрон приземлился!")
            except Exception as e:
                print(f"Ошибка при приземлении: {e}")

    return [lr, fb, ud, yv]

# Функция для отслеживания лица
def track_face(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    if len(faces) > 0:
        # Предполагаем, что мы отслеживаем первое обнаруженное лицо
        (x, y, w, h) = faces[0]
        return (x, y, w, h)  # Возвращаем координаты лица
    return None

try:
    while True:
        # Получение команд управления
        vals = getKeyboardInput()
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        # Получение видеопотока с камеры дрона
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))

        # Отслеживание лица
        face_position = track_face(frame)
        if face_position:
            x, y, w, h = face_position
            # Рисуем обводку вокруг лица
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Логика управления дроном для следования за лицом
            center_x = x + w // 2
            if center_x < 320:  # Если лицо слева от центра
                vals[0] = -50  # Двигаться влево
            elif center_x > 320:  # Если лицо справа от центра
                vals[0] = 50  # Двигаться вправо
            else:
                vals[0] = 0  # Остановить движение влево/вправо

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
qqq