import cv2
import urllib.request
import numpy as np
import time
from collections import deque
import threading
import sys

class cam_URL:
    def __init__(self, url, name='CAM', target_fps=30):
        self.url = url
        self.target_fps = target_fps
        self.name = name
        self.real_fps = deque([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.video_capt = cv2.VideoCapture(self.url)
        self.start_time = 0
        self.elapsed_time = 0
        self.frame_count = 0
        self.delay_frame = 0.0
        self.average_fps = 0
        self.ex_event = threading.Event()
        self.ex_event.clear()

        if self.isOpen():
            pass
        else:
            print("ERR connect "+self.name)
            exit()

    def isOpen(self):
        res = False
        if self.video_capt.isOpened():
            res = True
        return res

    def get_frame(self, isGetFPS=True):
        self.img_resp = urllib.request.urlopen(self.url)
        self.imgnp = np.array(bytearray(self.img_resp.read()), dtype=np.uint8)
        self.frame = cv2.imdecode(self.imgnp, -1)
        self.frame_count += 1

        # cv2.imshow(self.name, self.frame)
        if isGetFPS:
            pass
            # self.get_instant_fps()
            # self.get_aver_fps()

        return self.frame

    def get_instant_fps(self):
        self.elapsed_time = time.time() - self.start_time
        self.start_time = time.time()
        self.real_fps.popleft()
        self.real_fps.append(1/self.elapsed_time)
        print("instant FPS "+self.name+': ', self.real_fps[-1])
        return self.real_fps[-1]

    def get_aver_fps(self):
        self.average_fps = sum(self.real_fps)/len(self.real_fps)
        print("aver FPS "+self.name+': ', self.average_fps)
        return self.average_fps





def get_move():
    while True:
        try:
            cam_move.get_frame()
            cv2.imshow(cam_move.name, cam_move.frame)
        except:
            print('fuck move')

        key = cv2.waitKey(1)
        if key == ord('q'):
            cam_move.ex_event.set()
            break


def get_sens():
    while True:
        try:
            cam_sens.get_frame()
            cv2.imshow(cam_sens.name, cam_sens.frame)
        except:
            print('fuck sens')

        key = cv2.waitKey(1)
        if key == ord('q'):
            cam_sens.ex_event.set()
            break


if __name__ == '__main__':
    cam_move = cam_URL('http://192.168.4.2/cam-lo.jpg', 'MOVE', 2)
    cam_sens = cam_URL('http://192.168.4.3/cam-lo.jpg', "SENS", 30)
    
    th_move = threading.Thread(target=get_move, args=(), daemon=True)
    th_move.start()

    th_sens = threading.Thread(target=get_sens, args=(), daemon=True)
    th_sens.start()

    while True:
        time.sleep(.1)

        if cam_move.ex_event.is_set() or cam_sens.ex_event.is_set():
            break

    th_move.join()
    th_sens.join()
    sys.exit(0)
