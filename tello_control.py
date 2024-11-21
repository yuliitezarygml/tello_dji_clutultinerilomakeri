from djitellopy import Tello
import keyboard_control as kc
import time

kc.init()
drone = Tello()
drone.connect()
print(f"Уровень заряда батареи: {drone.get_battery()}%")

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
        drone.land()
        time.sleep(3)
    if kc.getKey("e"):
        drone.takeoff()

    return [lr, fb, ud, yv]

while True:
    vals = getKeyboardInput()
    drone.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    time.sleep(0.05)
