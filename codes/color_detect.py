import cv2
import numpy as np
import utils
import time

font = cv2.FONT_HERSHEY_SIMPLEX

class ColorDetect:
    def __init__(self):
        self.target_H = 0
        self.target_S = 0
        self.target_V = 0
        self.gray_frame = None
        self.hsv_frame = None
        self.target_x = None
        self.target_y = None
        self.is_detect = False
    
    def mouse_click(self, event, x, y, flags, para):
        if event == cv2.EVENT_LBUTTONDOWN:
            print ('PIX: ', x, y)
            print ('GRAY: ', self.gray_frame[y, x])
            print ('HSV: ', self.hsv_frame[y, x])

    def btn_click(self):
        print("ha")

    
    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max):
        src_frame = img
        result = src_frame
        self.gray_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2GRAY)
        self.hsv_frame = cv2.cvtColor(src_frame, cv2.COLOR_BGR2HSV)
        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(self.hsv_frame, low_color, high_color)
        mask_color = cv2.medianBlur(mask_color, 7)
        cv2.imshow("mask", mask_color)
        cnts, hierarchy = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        bx = 0
        by = 0
        bw = 0
        bh = 0
        for cnt in cnts:
            (x, y, w, h) = cv2.boundingRect(cnt)
            if w * h > bw * bh:
                bx, by, bw, bh = x, y, w, h

        
        cv2.rectangle(result, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)  
        cv2.putText(result, 'target', (bx, by - 5), font, 0.7, (0, 0, 255), 2)
          
     
def nothing(x):
    pass


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cd = ColorDetect()
    cv2.namedWindow('img')
    cv2.createTrackbar("H_MIN","img",0,180,nothing)
    cv2.createTrackbar("H_MAX","img",20,180,nothing)
    cv2.createTrackbar("S_MIN","img",60,255,nothing)
    cv2.createTrackbar("S_MAX","img",160,255,nothing)
    cv2.createTrackbar("V_MIN","img",30,255,nothing)
    cv2.createTrackbar("V_MAX","img",160,255,nothing)
    cv2.setMouseCallback("img", cd.mouse_click)

    while True:
        ret, frame = cap.read()
        #r_Img = frame
        r_Img = cv2.rotate(frame, cv2.ROTATE_180)
        h_min = cv2.getTrackbarPos("H_MIN","img")
        h_max = cv2.getTrackbarPos("H_MAX","img")
        s_min = cv2.getTrackbarPos("S_MIN","img")
        s_max = cv2.getTrackbarPos("S_MAX","img") 
        v_min = cv2.getTrackbarPos("V_MIN","img")
        v_max = cv2.getTrackbarPos("V_MAX","img")
        cd.update_frame(r_Img, h_min, h_max, s_min, s_max, v_min, v_max)
        cv2.imshow("img", r_Img)
        if cv2.waitKey(30) & 0xff == ord('q'):
            break
        if cv2.waitKey(30) & 0xff == ord('s'):
            cd.is_detect = True
    cap.release()
    cv2.destroyAllWindows()