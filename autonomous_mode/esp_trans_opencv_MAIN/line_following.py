from url_cam_class import cam_URL
import threading
import cv2
import time
import sys
import numpy as np
import serial
from pid import PID

cam_move = cam_URL('http://192.168.4.2/cam-lo.jpg', 'MOVE')
cam_sens = cam_URL('http://192.168.4.3/cam-lo.jpg', "SENS")

port = "COM7"
baudrate = 78600
ser = None


e_start = threading.Event()
e_start.clear()

move_timer = time.time()


def connect():
    global ser
    ser = None
    while not ser:
        try:
            ser = serial.Serial(port, baudrate=baudrate, timeout=1)
        except:
            print('oops')
            time.sleep(.5)

    while (not ser.isOpen()):
        ser.open()
        time.sleep(.5)
    time.sleep(0.1)


def send_com(l, r, arm):
    global ser
    msg = '#,{},{},{},;\n'.format(l, r, arm)
    try:
        ser.write(msg.encode())
        print('SEND:', msg)
    except:
        print('pizdec')
        connect()


def prepare(frame):
    frame = cv2.medianBlur(frame, 45)
    return (frame)


def get_move():
    try:
        cam_move.get_frame(False)
        # cv2.imshow(cam_move.name, cam_move.frame)
    except:
        print('ERR receive MOVE frame')
        exit(-1)
    return cam_move.frame


def move(move_timer, ser):
    dt = 0.06
    l, r = 0, 0
    kp = 0.3  # 0.3q
    ki = 0.3  # kp*2*dt
    kd = 0.0  # kp/8/dtq
    left_wheel = PID(kp, ki, kd, dt)
    right_wheel = PID(kp, ki, kd, dt)
    while True:
        if time.time() - move_timer > dt:
            move_timer = time.time()
            cadr = get_move()
            cv2.imshow("MOVE_0", cadr)
            key_s = cv2.waitKey(1)
            if key_s == ord('s'):
                e_start.set()
            elif key_s == ord('f'):
                e_start.clear()

            if e_start.is_set():

                cadr = get_move()

                cadr = cv2.flip(cadr, -1)
                cadr = prepare(cadr)
                width = cadr.shape[1]

                low_b = np.uint8([0, 0, 0])
                high_b = np.uint8([85, 85, 85])
                mask = cv2.inRange(cadr, low_b, high_b)

                contours, hierarchy = cv2.findContours(
                    mask, 1, cv2.CHAIN_APPROX_NONE)
                if len(contours) > 0:
                    c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        # print("CX : "+str(cx)+"  CY : "+str(cy))
                        # print(cx)
                        if cx > width // 2 + width/3.5:
                            left_wheel.calc(cx - width // 2)
                            right_wheel.calc(-(cx - width // 2))
                            right_wheel.res = 10
                        elif cx > width // 2 + 5:
                            left_wheel.calc(cx - width // 2)
                            right_wheel.calc(-(cx - width // 2))
                            # print("Turn RIGFT")
                        elif cx < width // 2 - width/3.5:
                            right_wheel.calc(width // 2 - cx)
                            left_wheel.calc(-(width // 2 - cx))
                            left_wheel.res = 10
                        elif cx < width // 2 - 5:
                            right_wheel.calc(width // 2 - cx)
                            left_wheel.calc(-(width // 2 - cx))
                            # print("Turn LEFT")
                        else:
                            left_wheel.calc(-1)
                            right_wheel.calc(-1)
                            # print('IDEAL')
                        cv2.circle(cadr, (cx, cy), 3, (0, 0, 255), -1)
                        cv2.drawContours(cadr, c, -1, (0, 255, 0), 1)
                    else:
                        print("Doesnt have M")
                        l, r = 0, 0
                else:
                    print("I don't see the line")
                    l, r = 0, 0
                l, r = left_wheel.res, right_wheel.res
                # send_com(l, -r, 0)
                msg = '#,{},{},{},;q'.format(l, r, 0)
                try:
                    ser.write(msg.encode())
                    ser.flush()
                    # print('SEND:', msg)
                except:
                    print('pizdec')
                    connect()
                # print(ser.readline())

                # cv2.imshow("Mask", mask)q
                cv2.imshow("MOVE", cadr)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    cam_move.ex_event.set()
                    break


if __name__ == '__main__':
    connect()

    th_move = threading.Thread(
        target=move, args=(move_timer, ser,), daemon=True)
    th_move.start()

    while True:
        time.sleep(.01)
        if cam_move.ex_event.is_set() or cam_sens.ex_event.is_set():
            break

    th_move.join()
    sys.exit(0)
