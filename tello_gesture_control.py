import cv2
from djitellopy import Tello
import time

# Инициализация дрона
drone = Tello()
drone.connect()
print(f"Уровень заряда батареи: {drone.get_battery()}%")
drone.streamon()

# Скорость движения дрона
SPEED = 50

# Функция для обработки нажатий клавиш
def get_keyboard_input():
    lr, fb, ud, yv = 0, 0, 0, 0
    key = cv2.waitKey(1) & 0xFF

    if key == ord('a'):  # Влево
        lr = -SPEED
    elif key == ord('d'):  # Вправо
        lr = SPEED

    if key == ord('w'):  # Вперёд
        fb = SPEED
    elif key == ord('s'):  # Назад
        fb = -SPEED

    if key == ord('r'):  # Вверх
        ud = SPEED
    elif key == ord('f'):  # Вниз
        ud = -SPEED

    if key == ord('j'):  # Поворот влево
        yv = -SPEED
    elif key == ord('l'):  # Поворот вправо
        yv = SPEED

    if key == ord('t'):  # Взлёт
        drone.takeoff()
    elif key == ord('g'):  # Посадка
        drone.land()

    return [lr, fb, ud, yv]

try:
    while True:
        # Получение команд с клавиатуры
        vals = get_keyboard_input()
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        # Получение кадра с камеры дрона
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow("Tello Camera", frame)

        # Выход из программы при нажатии клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pass
finally:
    drone.streamoff()
    cv2.destroyAllWindows()
