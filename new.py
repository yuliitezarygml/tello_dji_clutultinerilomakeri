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
        try:
            drone.land()
            print("Дрон приземлился.")
            time.sleep(3)
        except Exception as e:
            print(f"Ошибка при посадке: {e}")

    if kc.getKey("e"):
        try:
            print("Попытка взлёта...")
            drone.takeoff()
            print("Дрон взлетел.")
        except Exception as e:
            print(f"Ошибка при взлёте: {e}")

    return [lr, fb, ud, yv]

try:
    while True:
        # Получение команд управления
        vals = getKeyboardInput()
        drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        # Получение видеопотока с камеры дрона
        frame = drone.get_frame_read().frame
        frame = cv2.resize(frame, (640, 480))
        cv2.imshow("Tello Camera", frame)

        # Выход из программы при нажатии клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    print("Программа остановлена пользователем.")
finally:
    try:
        drone.streamoff()
    except Exception as e:
        print(f"Ошибка при остановке видеопотока: {e}")
    cv2.destroyAllWindows()
