'''
Запуск - в холостую получаем кадры с камер
После на нажатия S, начинаем работу:
    сенс аруки 10
        если был, то начинаем ехать до тех пор, пока не сенс аруки 20
    сенс красного
        если сенс, то не меняем статус, пока не сенс аруки 2
'''

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
baudrate = 115200
ser = None

e_start = threading.Event()
e_start.clear()

e_arm = threading.Event()
e_arm.clear()

e_aruco_init = threading.Event()
e_aruco_init.clear()

e_pomidor = threading.Event()
e_pomidor.clear()

e_hotim_otkl = threading.Event()
e_hotim_otkl.clear()

move_timer = time.time()
sens_timer = time.time()


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
        cam_move.get_frame(True)
        # cv2.imshow(cam_move.name, cam_move.frame)
    except:
        print('ERR receive MOVE frame')
        exit(-1)
    return cam_move.frame


def get_sense():
    try:
        cam_sens.get_frame(True)
    except:
        print('ERR receive SENS frame')
        exit(-1)
    return cam_sens.frame


def move(move_timer, ser):
    dt = 0.06  #0.06
    l, r = 0, 0
    kp = 0.2  # 0.3q
    ki = 0.6  # kp*2*dt
    kd = 0.0  # kp/8/dtq
    left_wheel = PID(kp, ki, kd, dt)
    right_wheel = PID(kp, ki, kd, dt)

    low_b = np.uint8([0, 0, 0])
    high_b = np.uint8([85, 85, 85])
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

            if e_start.is_set() or True:
                if e_aruco_init.is_set():

                    # cadr = get_move()

                    cadr = cv2.flip(cadr, -1)
                    cadr = prepare(cadr)
                    width = cadr.shape[1]

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
                                right_wheel.res = 11
                            elif cx > width // 2 + 5:
                                left_wheel.calc(cx - width // 2)
                                right_wheel.calc(-(cx - width // 2))
                                # print("Turn RIGFT")
                            elif cx < width // 2 - width/3.5:
                                right_wheel.calc(width // 2 - cx)
                                left_wheel.calc(-(width // 2 - cx))
                                left_wheel.res = 11
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
                            l, r = left_wheel.res, right_wheel.res
                        else:
                            print("Doesnt have M")
                            l, r = 10, 10
                    else:
                        print("I don't see the line")
                        l, r = 10, 10
                    
                    # send_com(l, -r, 0)
                    msg = '#,{},{},{},;q'.format(l, r, int(e_arm.is_set()))
                    byte_msg = msg.encode()
                    print(byte_msg)
                    try:
                        ser.write(byte_msg)
                        ser.flush()
                        print('SEND:', msg)
                        print()
                        pass
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


def sens(sens_timer):
    dt = 0.06
    l_h, l_s, l_v = 170, 64, 64
    u_h, u_s, u_v = 180, 255, 255
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    parameters = cv2.aruco.DetectorParameters()
    while True:
        if time.time() - sens_timer > dt:
            sens_timer = time.time()
            frame = get_sense()
            cv2.imshow("SENS_0", frame)

            if e_start.is_set() or True:

                corners, ids, rejected = cv2.aruco.ArucoDetector(
                    dictionary, parameters).detectMarkers(frame)
                if ids is None:
                    pass
                else:
                    # print(ids)
                    marker_corners = corners[0][0]
                    x, y, w, h = cv2.boundingRect(marker_corners)
                    cv2.rectangle(frame, (x, y), (x + w, y + h),
                                  (0, 255, 255), 5)
                    if ids[0] == 20 and not e_aruco_init.is_set():
                        e_aruco_init.set()
                    if ids[0] == 10 and e_aruco_init.is_set():
                        e_aruco_init.clear()

                    if ids[0] == 2 and e_pomidor.is_set():
                        e_pomidor.clear()
                        # e_hotim_otkl.set()
                    if ids[0] == 1:
                        # проехали какой-то овосч
                        pass
                        
                if e_pomidor.is_set():
                    e_arm.set()
                else:
                    e_arm.clear()
                        
                # frame = cv2.flip(frame, 0)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # hsv
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # gray
                mask = cv2.inRange(hsv, l_b, u_b)
                cnts, _ = cv2.findContours(
                    mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if area > 400:
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
                        M = cv2.moments(c)
                        cx = int(M["m10"]/M["m00"])
                        cy = int(M["m01"]/M["m00"])

                        cv2.circle(frame, (cx, cy), 2, (255, 255, 255), -1)
                        cv2.putText(frame, "pomidor", (cx-20, cy-20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                        e_pomidor.set()
                    else:
                        if e_hotim_otkl.set():
                            e_hotim_otkl.clear()
                            e_pomidor.clear()
                        pass

                # res = cv2.bitwise_and(frame, frame, mask=mask)

                cv2.imshow("mask", mask)
                # cv2.imshow("res", res)

                cv2.imshow("SENS", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            cam_sens.ex_event.set()
            break


if __name__ == '__main__':
    connect()

    th_move = threading.Thread(
        target=move, args=(move_timer, ser,), daemon=True)
    th_move.start()

    th_sens = threading.Thread(target=sens, args=(sens_timer,), daemon=True)
    th_sens.start()

    while True:
        time.sleep(.01)
        if cam_move.ex_event.is_set() or cam_sens.ex_event.is_set():
            break

    th_move.join()
    th_sens.join()
    sys.exit(0)
