import cv2
import urllib.request
import numpy as np
import time

url_move = 'http://192.168.1.12/cam-lo.jpg'
url_sens = 'http://192.168.1.14/cam-lo.jpg'
cam_move = cv2.VideoCapture(url_move)
cam_sens = cv2.VideoCapture(url_sens)
if not cam_move.isOpened():
    print("Failed to open the IP camera SENS stream")
    exit()
if not cam_sens.isOpened():
    print("Failed to open the IP camera MOVE stream")
    exit()

# Установите желаемую частоту кадров для каждой камеры (в кадрах в секунду)
desired_fps_move = 1
desired_fps_sens = 2

# Переменные для хранения времени начала и конца кадра
start_time_move = time.time()
start_time_sens = time.time()

# Переменные для хранения количества кадров
frame_count_move = 0
frame_count_sens = 0

while True:
    # Получение кадра с камеры MOVE
    img_resp_move = urllib.request.urlopen(url_move)
    imgnp_move = np.array(bytearray(img_resp_move.read()), dtype=np.uint8)
    frame_move = cv2.imdecode(imgnp_move, -1)

    # Показать кадр
    cv2.imshow('MOVE', frame_move)

    frame_count_move += 1
    elapsed_time_move = time.time() - start_time_move

    # Получение кадра с камеры SENS
    img_resp_sens = urllib.request.urlopen(url_sens)
    imgnp_sens = np.array(bytearray(img_resp_sens.read()), dtype=np.uint8)
    frame_sens = cv2.imdecode(imgnp_sens, -1)

    # Показать кадр
    cv2.imshow('SENS', frame_sens)

    frame_count_sens += 1
    elapsed_time_sens = time.time() - start_time_sens
    
    
    # Если прошла одна секунда, пересчитываем FPS и устанавливаем задержку перед получением следующего кадра
    if elapsed_time_move >= 1.0:
        fps_move = frame_count_move / elapsed_time_move
        print("MOVE FPS:", fps_move, end='    ')
        start_time_move = time.time()
        frame_count_move = 0

        # Вычисляем задержку перед следующим кадром, чтобы соблюсти желаемую частоту кадров
        delay_move = 1.0 / desired_fps_move - elapsed_time_move
        if delay_move > 0:
            time.sleep(delay_move)

    if elapsed_time_sens >= 1.0:
        fps_sens = frame_count_sens / elapsed_time_sens
        print("SENS FPS:", fps_sens)
        start_time_sens = time.time()
        frame_count_sens = 0

        # Вычисляем задержку перед следующим кадром, чтобы соблюсти желаемую частоту кадров
        delay_sens = 1.0 / desired_fps_sens - elapsed_time_sens
        if delay_sens > 0:
            time.sleep(delay_sens)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cam_move.release()
cv2.destroyAllWindows()
