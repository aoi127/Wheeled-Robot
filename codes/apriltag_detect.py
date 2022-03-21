import cv2
import apriltag
import sys
import numpy as np
import time
from multiprocessing import Process
from multiprocessing.managers import BaseManager

if __name__ == '__main__':
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11 tag25h9')) #apriltag信息
    cap = cv2.VideoCapture('1_yolov5.avi')
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('1_full_detect.avi',fourcc, 20.0, (640,480),True)
    while(cap.isOpened()):
        ret, frame = cap.read()
        energy_block_is_detected = 0
        bomb_is_detected = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)   #灰度图像
        tags = at_detector.detect(gray)

        for tag in tags:
            # print(tag.tag_id)
            if tag.tag_id == 0:
                bomb_is_detected = 1
            if tag.tag_id == 1:
                energy_block_is_detected = 1
        
        if energy_block_is_detected:
            for tag in tags:
                if tag.tag_id == 1:
                    cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) # left-top
                    cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) # right-top
                    cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) # right-bottom
                    cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2) # left-bottom

        if bomb_is_detected:
            for tag in tags:
                if tag.tag_id == 0:
                    cv2.circle(frame, tuple(tag.corners[0].astype(int)), 4, (0, 0, 255), 2) # left-top
                    cv2.circle(frame, tuple(tag.corners[1].astype(int)), 4, (0, 0, 255), 2) # right-top
                    cv2.circle(frame, tuple(tag.corners[2].astype(int)), 4, (0, 0, 255), 2) # right-bottom
                    cv2.circle(frame, tuple(tag.corners[3].astype(int)), 4, (0, 0, 255), 2) # left-bottom
        out.write(frame)
        cv2.imshow("img", frame)
        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()
