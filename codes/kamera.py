# -*- coding:utf-8
import numpy as np
import multiprocessing
from multiprocessing import Process, Queue, Manager
import random
import cv2

def Camera(que):
    cap= cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)
    # cap.set(3,320)
    # cap.set(4,240)
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('testwrite.avi',fourcc, 30.0, (640,480),True)

    while 1:
        ret,img = cap.read()
        #out.write(img)
        lenth = que.qsize()
        if lenth > 2:
            for i in range(lenth-2):
                frame = que.get()   #清除缓存
        img = cv2.rotate(img, cv2.ROTATE_180)
        que.put(img)


if __name__ == '__main__':
    manager = Manager()
    q = manager.Queue()
    camera = Process(target=Camera, args=(q,))
    camera.start()
    while True:
        frame = q.get()
        cv2.circle(frame, (220, 0), 4, (255, 0, 0), 2)
        cv2.circle(frame, (420, 0), 4, (255, 0, 0), 2)
        for i in range(0, 481):
            cv2.circle(frame, (320, i), 4, (255, 0, 0), 2) 
        for i in range(0, 641):
            cv2.circle(frame, (i, 420), 4, (255, 0, 0), 2) 
        cv2.imshow('img',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # out.release()

