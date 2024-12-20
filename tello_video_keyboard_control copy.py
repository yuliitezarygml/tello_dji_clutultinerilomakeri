import cv2
import numpy as np
from djitellopy import Tello
import keyboard_control as kc
import time

# Инициализация модуля управления клавиатурой
kc.init()

# Инициализация дронаm
drone = Tello()
drone.connect()
print(f"Уровень заряда батареи: {drone.get_battery()}%")
drone.streamon()

# Загрузка каскадных классификаторов для обнаружения лиц и тел
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')


# Функция для обработки команд с клавиатуры
def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50

    if kc.getKey("LEFT"):
        lr = -speed
    elif kc.getKey("RIGHT"):
        lr = speed

    if kc.getKey("UP"):
        fb = speed
    elif kc.getKey("DOWN"):
        fb = -speed

    if kc.getKey("w"):
        ud = speed
    elif kc.getKey("s"):
        ud = -speed

    if kc.getKey("a"):
        yv = -speed
    elif kc.getKey("d"):
        yv = speed

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


# Функция для отслеживания лиц, тел и других объектов
def track_objects(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Обнаружение лиц
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)
    # Обнаружение тел
    bodies = body_cascade.detectMultiScale(gray, 1.1, 4)

    return faces, bodies  # Возвращаем обнаруженные лица и тела


# Функция для обнаружения контуров
def detect_contours(frame):
    # Преобразование в оттенки серого
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Применение размытия
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # Применение порогового значения
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
    # Нахождение контуров
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours


try:
    while True:
        # Получение команд управления
        vals = getKeyboardInput()
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        # Получение видеопотока с камеры дрона
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))

        # Отслеживание объектов
        faces, bodies = track_objects(frame)

        # Рисуем обводки вокруг лиц
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Рисуем обводки вокруг тел
        for (x, y, w, h) in bodies:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

        # Обнаружение контуров
        contours = detect_contours(frame)
        for contour in contours:
            # Рисуем обводку вокруг каждого контура
            cv2.drawContours(frame, [contour], -1, (0, 0, 255), 2)  # Красный цвет для контуров

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