import sys
import cv2
import apriltag
import numpy as np
import uptech
import time
from multiprocessing import Process, Manager, Value, Array
import kamera
import random
import copy
import yolov5

#import matplotlib.pyplot as plt

BOMB = 0
ENERGY = 1
io_front = 3
io_back = 4
io_left = 0
io_right = 6
io_left_front = 2
io_right_front = 7
io_left_back = 1
io_right_back = 5
adc_left = 1
adc_right = 3
adc_front = 4
adc_back_left = 0
adc_back_right = 2
adc_threshold = [250,200,235,200,190]

t_dashback = 1.4
t_turn_180 = 0.46
t_turn_135 = 0.35
t_turn_90 = 0.25
t_turn_45 = 0.15
t_a_little_up = 0.1
t_a_little_back = 0.2
t_delay = 0.1

def nothing(*arg):
    pass

class MyController:

##初始化
    def __init__(self):
        self.energy_block_is_detected = Value('i', 0)
        self.bomb_is_detected = Value('i', 0)
        self.normal_block_is_detected = Value('i', 0)
        self.edge_flag = Value('i', 0) # 0 for close, 1 for attack, 2 for around edge
        self.energy_block_down_num = Value('i', 0)
        self.energy_block = ENERGY
        self.bomb = BOMB
        self.up=uptech.UpTech()
        self.up.ADC_IO_Open()
        self.up.CDS_Open()
        self.up.CDS_SetMode(1, 1)
        self.up.CDS_SetMode(2, 1)

        self.adc_data = Array('i', range(5))
        self.io_data = Array('i', range(8))

        self.at_detector = apriltag.Detector(
                    apriltag.DetectorOptions(families='tag36h11 tag25h9')) #apriltag信息
        self.frame = []

        self.manager = Manager()
        self.frame_queue = self.manager.Queue()
        self.p_camera = Process(target=kamera.Camera, args=(self.frame_queue,))
        self.p_camera.start()

        self.move_flag = Value('i', 0) # 0 = stop, 1 = up, 2 = back, 
                                       # 3 = left, 4 = right , 5 = attack

        self.p_update_io_adc_data = Process(target=self.update_io_adc_datas)
        self.p_update_io_adc_data.start()

        self.energy_block_x = Value('i', -1)
        self.bomb_x = Value('i', -1)
        self.normal_block_x = Value('i', -1)

        self.p_edge_detect = Process(target=self.edge_detect)

        self.last_move_choice = 10
        self.just_edge = Value('i', 0)
        time.sleep(1)

        # self.car_cascade = cv2.CascadeClassifier('/home/pi/uptech_star/cascades/car_cascade4.0.xml')

    def update_io_adc_datas(self):
         while True:
            end = 10
            mid = int(end / 2)
            io_sum = np.zeros((8,), dtype=np.int)
            adc_inputs = np.zeros((5,end), dtype=np.int)

            for epoch in range(end):
                io_all_input = self.up.ADC_IO_GetAllInputLevel()
                io_array = '{:08b}'.format(io_all_input)
                adc_input = self.up.ADC_Get_All_Channle()
                for index,value in enumerate(io_array):
                    io = ((int)(value) + 1) % 2
                    io_sum[index] = io_sum[index] + io
                for index in range(5):
                    adc_inputs[index, epoch] = adc_input[index]
            
            adc_inputs = np.sort(adc_inputs, axis=1)

            for i in range(8):
                # print('iodata:',io_sum)
                if io_sum[i] > mid:
                    self.io_data[7-i] = 1
                else:
                    self.io_data[7-i] = 0

            for i in range(5):
                # print(i,':',adc_inputs[i])
                self.adc_data[i] = adc_inputs[i, mid]
    
    def update_frame(self, queue):
        # print('update_frame')
        self.frame = queue.get()
    
    def first_check(self):
        # print(self.io_data)
        if self.io_data[io_right] and self.io_data[io_front]:
            return True

    def move_up(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 1
        self.up.CDS_SetSpeed(1,500)
        print('    move_up ')
        self.up.CDS_SetSpeed(2,-500)

    def move_back(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 2
        self.up.CDS_SetSpeed(1,-500)
        print('    move_back')
        self.up.CDS_SetSpeed(2,500)

    def turn_left_fast(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 3
        self.up.CDS_SetSpeed(1,-1023)
        print('    turn_left_fast')
        self.up.CDS_SetSpeed(2,-1023)

    def turn_right_fast(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 4
        self.up.CDS_SetSpeed(1,1023)
        print('    turn_right_fast')
        self.up.CDS_SetSpeed(2,1023)
    
    def move_stop(self):
        while self.edge_flag.value == 2:
            pass
        self.move_flag.value = 0
        self.up.CDS_SetSpeed(1,0)
        print('    move_stop')
        self.up.CDS_SetSpeed(2,0)

    def attack(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        # print('attack')
        got_something = False
        self.up.CDS_SetSpeed(1,350)
        self.move_flag.value = 5
        self.up.CDS_SetSpeed(2,-350)
        self.edge_flag.value = 1
        print("attacking...")
        while self.edge_flag.value:
            if self.io_data[io_front]:
                got_something = True
            elif got_something:
                if self.edge_flag.value == 1:
                    self.edge_flag.value = 0
            pass
        print("stop_attack")
        self.move_stop()
        '''self.move_stop()
        self.move_back()
        time.sleep(t_a_little_back)'''

    def turn_left_slow(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 3
        self.up.CDS_SetSpeed(1,-300)
        print('    turn_left_slow')
        self.up.CDS_SetSpeed(2,-300)
    
    def turn_right_slow(self):
        while self.move_flag.value != 0 or self.edge_flag.value == 2:
            pass
        self.move_flag.value = 4
        self.up.CDS_SetSpeed(1,300)
        print('    turn_right_slow')
        self.up.CDS_SetSpeed(2,300)

    def edge_detect(self):
        while True:
            # t_start = time.time()
            choice = 0
            if not self.io_data[io_left_front]:
                if not self.io_data[io_right_front]:
                    if not self.io_data[io_right_back]:
                        if not self.io_data[io_left_back]:
                            choice = -1
                        else:
                            choice = 12
                    elif not self.io_data[io_left_back]:
                        choice = 11
                    else:
                        choice = 1
                elif not self.io_data[io_left_back]:
                    if not self.io_data[io_right_back]:
                        choice = 10
                    else:
                        choice = 3
                elif not self.io_data[io_right_back]:
                    choice = -1
                else:
                    choice = 5
            elif not self.io_data[io_right_front]:
                if not self.io_data[io_right_back]:
                    if not self.io_data[io_left_back]:
                        choice = 9
                    else: 
                        choice = 4
                elif not self.io_data[io_left_back]:
                    choice = -1
                else:
                    choice = 6
            elif not self.io_data[io_right_back]:
                if not self.io_data[io_left_back]:
                    choice = 2
                else:
                    choice = 7
            elif not self.io_data[io_left_back]:
                choice = 8
            else:
                choice = 0

            if choice >= 1:
                self.just_edge.value = 1

            if choice == 1:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                if self.move_flag.value == 5 :
                    self.energy_block_down_num.value = self.energy_block_down_num.value + 1 
                self.up.CDS_SetSpeed(1,-1023)
                self.up.CDS_SetSpeed(2,1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_back] or not self.io_data[io_right_back]:
                        break 
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)
                time.sleep(0.2)          
                self.edge_flag.value = 0
            elif choice == 2:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,-1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_front] or not self.io_data[io_right_front]:
                        break 
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0) 
                time.sleep(0.2)            
                self.edge_flag.value = 0
            elif choice == 3:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,0)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)  
                time.sleep(0.2)           
                self.edge_flag.value = 0
            elif choice == 4:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,-1023)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)    
                time.sleep(0.2)         
                self.edge_flag.value = 0
            elif choice == 5 or choice == 11:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                if self.move_flag.value == 5 :
                    self.energy_block_down_num.value = self.energy_block_down_num.value + 1 
                self.up.CDS_SetSpeed(1,-1023)
                self.up.CDS_SetSpeed(2,1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_back] or not self.io_data[io_right_back]:
                        break 
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,1023)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)     
                time.sleep(0.2)        
                self.edge_flag.value = 0
            elif choice == 6 or choice == 12:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                if self.move_flag.value == 5 :
                    self.energy_block_down_num.value = self.energy_block_down_num.value + 1 
                self.up.CDS_SetSpeed(1,-1023)
                self.up.CDS_SetSpeed(2,1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_back] or not self.io_data[io_right_back]:
                        break 
                self.up.CDS_SetSpeed(1,-1023)
                self.up.CDS_SetSpeed(2,-1023)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)  
                time.sleep(0.2)           
                self.edge_flag.value = 0
            elif choice == 7 or choice == 9:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,-1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_front] or not self.io_data[io_right_front]:
                        break 
                self.up.CDS_SetSpeed(1,-1023)
                self.up.CDS_SetSpeed(2,-1023)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)    
                time.sleep(0.2)         
                self.edge_flag.value = 0
            elif choice == 8 or choice == 10:
                print('edge_detect ', choice)
                self.edge_flag.value = 2
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,-1023)
                for i in range(0,10):
                    time.sleep(0.01)
                    if not self.io_data[io_left_front] or not self.io_data[io_right_front]:
                        break 
                self.up.CDS_SetSpeed(1,1023)
                self.up.CDS_SetSpeed(2,1023)
                time.sleep(0.1)
                self.up.CDS_SetSpeed(1,0)
                self.up.CDS_SetSpeed(2,0)    
                time.sleep(0.2)         
                self.edge_flag.value = 0

            # t_end = time.time()
            # print("edge_detect consume = ", t_end - t_start)

    def color_filter(self, img, h1, h2, s1, s2, v1, v2, flag):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        low_color = np.array([h1, s1, v1])
        high_color = np.array([h2, s2, v2])
        mask_color = cv2.inRange(hsv, low_color, high_color)
        mask_color = cv2.medianBlur(mask_color, 7)
        if(flag == 1):
            #cv2.imshow("mask", mask_color)
            img1 = cv2.bitwise_and(img, img, mask = mask_color)
            #cv2.imshow("img1", img1)
            self.line_detect_possible_demo(img1)
        cnts, hierarchy = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        bx, by, bw, bh = 0, 0, 0, 0
        for cnt in cnts:
            (x, y, w, h) = cv2.boundingRect(cnt)
            #print(w / h)
            if((flag == 0 and w / h <= 1.3 and w * h > bw * bh) or (flag == 1 and w > bw and w / h > 1.2 and w > 20)):
                #print(w / h)
                bx, by, bw, bh = x, y, w, h
                #print(bx, by, bw, bh)
        return bx, by, bw, bh

    def line_detect_possible_demo(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)  # apertureSize是sobel算子大小，只能为1,3,5，7
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 200, minLineLength=20,maxLineGap=10)  #函数将通过步长为1的半径和步长为π/180的角来搜索所有可能的直线
        if not(lines is None):
            for line in lines:
                #print(type(line))   #多维数组
                x1,y1,x2,y2 = line[0]
                cv2.line(image,(x1,y1),(x2,y2),(0,0,255),2)
        #cv2.imshow("line_detect_possible_demo",image)

    def yolov5(self):
        yolov5.detect()

    def get_block_location(self):
        #start_time = time.time()
        self.update_frame(self.frame_queue)
        self.yolov5()
        #self.line_detect_possible_demo(self.frame)

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        after_filt = copy.deepcopy(self.frame)

        bx, by, bw, bh = self.color_filter(self.frame, 0, 20, 80, 160, 30, 160, 0)

        if bh > 35 and bw > 25:
            cv2.rectangle(self.frame, (bx, by), (bx + bw, by + bh), (255, 0, 255), 2)  
            cv2.putText(self.frame, 'block', (bx, by - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            self.normal_block_is_detected.value = 1
            self.normal_block_x.value = (int)(bx + bw / 2)
            #cv2.rectangle(after_filt, (bx - 10, by - 10), (bx + bw + 10, by + bh + 10), (255, 255, 255), -1)
        else:
            self.normal_block_is_detected.value = 0
            self.normal_block_x.value = -1

        #print(bx, by, bw, bh)

        # print('get_block_location')
        self.energy_block_is_detected.value = 0
        self.energy_block_x.value = -1
        self.bomb_is_detected.value = 0
        self.bomb_x.value = -1
        tags = self.at_detector.detect(gray)

        energy_block_h_max = 0
        energy_block_y2 = 0
        energy_block_w = 0
        bomb_h_max = 0
        bomb_y2 = 0
        for tag in tags:
            x1, y1, x2, y2 = self.regularization(tag)
            #cv2.rectangle(after_filt, (x1 - 20 , y1 - 20), (x2 + 20, y2 + 20), (255, 255, 255), -1)
            #print("tag.corners",tag.corners)
            if tag.tag_id == self.energy_block:
                self.energy_block_is_detected.value = 1
                cv2.circle(self.frame, tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2) 
                cv2.circle(self.frame, tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2) 
                cv2.circle(self.frame, tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2) 
                cv2.circle(self.frame, tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)
                energy_block_h = abs(y2 - y1)
                if(energy_block_h > energy_block_h_max): # find the block with the biggest height -- nearest
                    energy_block_h_max = energy_block_h
                    energy_block_y2 = y2
                    self.energy_block_x.value = (int)((x1 + x2) / 2)
                    energy_block_w = abs(x2 - x1)
            if tag.tag_id == self.bomb:
                self.bomb_is_detected.value = 1
                cv2.circle(self.frame, tuple(tag.corners[0].astype(int)), 4, (0, 0, 255), 2) 
                cv2.circle(self.frame, tuple(tag.corners[1].astype(int)), 4, (0, 0, 255), 2) 
                cv2.circle(self.frame, tuple(tag.corners[2].astype(int)), 4, (0, 0, 255), 2) 
                cv2.circle(self.frame, tuple(tag.corners[3].astype(int)), 4, (0, 0, 255), 2)
                bomb_h = abs(y2 - y1)
                if(bomb_h > bomb_h_max):
                    bomb_h_max = bomb_h
                    bomb_y2 = y2
                    self.bomb_x.value = (int)((x1 + x2) / 2)
        #print("bomb_h_max", bomb_h_max)
        if(bomb_h_max < 100): # do not want to avoid too far bomb
            self.bomb_is_detected.value = 0 
        end_time = time.time()
        #print("energy_block_h_max", energy_block_h_max)
        #print("self.bomb_x", self.bomb_x.value)
        #print("self.energy_block_x", self.energy_block_x.value)

        #ex, ey, ew, eh = self.color_filter(after_filt, 10, 170, 1, 254, 1, 130, 1)
        #cv2.rectangle(self.frame, (ex, ey), (ex + ew, ey + eh), (0, 255, 255), 2) 

        '''if by + bh < ey + eh:
            self.normal_block_is_detected.value = 0
            self.normal_block_x.value = -1
        if energy_block_y2 < ey + eh :
            self.energy_block_is_detected.value = 0
            self.energy_block_x.value = -1
        if bomb_y2 < ey + eh:
            self.bomb_is_detected.value = 0
            self.bomb_x.value = -1'''
        cv2.imshow("img", self.frame)
        #cv2.imshow("after_filt", after_filt)
        cv2.waitKey(1)
        #print(end_time - start_time)

    def regularization(self, tag):
        #print(tag.corners.astype(int))
        x1 = np.amin(tag.corners.astype(int), axis=0)[0]
        y1 = np.amin(tag.corners.astype(int), axis=0)[1]
        x2 = np.amax(tag.corners.astype(int), axis=0)[0]
        y2 = np.amax(tag.corners.astype(int), axis=0)[1]
        #print(x1, y1, x2, y2)
        return x1, y1, x2, y2

    def find_by_vision(self):
        print("find_by_vision")
        for chance in range(0, 2):
            self.get_block_location()
            if self.energy_block_is_detected.value:
                print("energy", self.energy_block_x.value)
                # return False
                if self.energy_block_x.value < 310:
                    self.move_stop()
                    self.turn_left_slow()
                    time.sleep(0.1)
                    self.move_stop()
                    self.last_move_choice = 10
                    return False
                elif self.energy_block_x.value > 360:
                    self.move_stop()
                    self.turn_right_slow()
                    time.sleep(0.1)
                    self.move_stop()
                    self.last_move_choice = 20
                    return False
                else:
                    if self.bomb_x.value > 360 or self.bomb_x.value < 310:
                        return True
                    return True

            elif self.normal_block_is_detected.value:
                print("normal", self.normal_block_x.value)
                if self.normal_block_x.value < 310:
                    self.move_stop()
                    self.turn_left_slow()
                    time.sleep(0.1)
                    self.move_stop()
                    return False
                elif self.normal_block_x.value > 360:
                    self.move_stop()
                    self.turn_right_slow()
                    time.sleep(0.1)
                    self.move_stop()
                    return False
                else:
                    if self.bomb_x.value > 360 or self.bomb_x.value < 310:
                        return True
        if self.last_move_choice >= 0 and self.last_move_choice <= 9:
            if self.just_edge.value == 0:
                choice = np.random.randint(3, 11)
            else:
                choice = np.random.randint(13, 21)
                self.just_edge.value = 0
        elif self.last_move_choice >= 11 and self.last_move_choice <= 19:
            if self.just_edge.value == 0:
                choice = np.random.randint(13, 21)
            else:
                choice = np.random.randint(3, 11)
                self.just_edge.value = 0
        elif self.last_move_choice == 10:
            choice = np.random.randint(13, 21)
        elif self.last_move_choice == 20:
            choice = np.random.randint(3, 11)
        print(choice)
        if choice >= 1 and choice <= 9:
            self.move_stop()
            self.turn_left_fast()
            time.sleep(0.1)
            self.move_stop()
            time.sleep(t_delay)
            self.last_move_choice = choice
            return False
        elif choice >= 11 and choice <= 19:
            self.move_stop()
            self.turn_right_fast()
            time.sleep(0.1)
            self.move_stop()
            time.sleep(t_delay)
            self.last_move_choice = choice
            return False
        elif choice == 10 or choice == 20:
            if not (self.bomb_is_detected.value == 1 and self.bomb_x.value > 100 and self.bomb_x.value < 540):
                self.move_stop()
                #time.sleep(0.1)
                #if not self.move_flag.value: 
                self.move_up()
                time.sleep(0.5)
                self.move_stop()
                time.sleep(t_delay)
                self.last_move_choice = choice
            else:
                print("BOMB DETECT!")
            return False

    def find_by_sensor(self):
        print('find_by_sensor')
        self.get_block_location()
        if self.io_data[io_front]:
            time.sleep(t_delay)
            self.get_block_location()
            if not (self.bomb_is_detected.value == 1 and self.bomb_x.value > 100 and self.bomb_x.value < 540):
                choice = 1
            else:
                print("BOMB DETECT!")
                self.move_stop()
                self.move_back()
                time.sleep(t_a_little_back)
                self.move_stop()
                choice = np.random.randint(3,5)
        elif self.io_data[io_back]:
            choice = 2
        elif self.io_data[io_left]:
            choice = 3
        elif self.io_data[io_right]:
            choice = 4
        elif self.energy_block_is_detected.value == 1 or self.normal_block_is_detected.value == 1:
            choice = 7
        else:
            choice = np.random.randint(5, 10)
                
        # choice = 7
        if choice == 1:
            self.last_move_choice = choice
            return True
        elif choice == 2:
            self.move_stop()
            self.turn_right_fast()
            time.sleep(t_turn_180)
            self.move_stop()
            self.last_move_choice = choice
            return False
        elif choice == 3:
            self.move_stop()
            self.turn_left_fast()
            time.sleep(t_turn_90)
            self.move_stop()
            self.last_move_choice = choice
            return False
        elif choice == 4:
            self.move_stop()
            self.turn_right_fast()
            time.sleep(t_turn_90)
            self.move_stop()
            self.last_move_choice = choice
            return False
        elif choice == 5:
            if self.last_move_choice == 6:
                return False
            self.move_stop()
            self.turn_right_fast()
            time.sleep(0.1)
            self.move_stop()
            self.last_move_choice = choice
            return False
        elif choice == 6:
            if self.last_move_choice == 5:
                return False
            self.move_stop()
            self.turn_left_fast()
            time.sleep(0.1)
            self.move_stop()
            self.last_move_choice = choice
            return False
        elif choice >= 7:
            time.sleep(t_delay)
            self.get_block_location()
            if self.energy_block_is_detected.value == 1 or self.normal_block_is_detected.value == 1:
                if not (self.bomb_is_detected.value == 1 and self.bomb_x.value > 100 and self.bomb_x.value < 540):
                    self.move_stop()
                    self.move_up()
                    time.sleep(0.5)
                    self.move_stop()
                    self.last_move_choice = choice
                else:
                    print("BOMB DETECT!")
            return False

    def upstage(self):
        self.up.CDS_SetSpeed(1, -1023)
        print('upstage')
        self.up.CDS_SetSpeed(2,1023)
        time.sleep(1)
        self.up.CDS_SetSpeed(1,-400)
        time.sleep(1)
        self.up.CDS_SetSpeed(1,0)
        self.up.CDS_SetSpeed(2,0)
        self.p_edge_detect.start()

    def upstage_adjust(self):
        while True:
            time.sleep(0.1)
            print('upstage_adjust')
            if not self.io_data[io_back] and self.adc_data[adc_back_right] > adc_threshold[adc_back_right] \
                and self.adc_data[adc_back_left] > adc_threshold[adc_back_left]:
                sum = 0
                for i in range(10):
                    sum = sum + self.adc_data[adc_back_left] * 0.9 - self.adc_data[adc_back_right]
                if np.abs(sum) < 100:
                    print("time to climb! : ", sum)
                    break
                elif sum < -100:
                    print("turn left a little : ", sum)
                    self.move_stop()
                    self.turn_left_fast()
                    time.sleep(0.01)
                    self.move_stop()
                elif sum > 100:
                    print("turn right a little : ", sum)
                    self.move_stop()
                    self.turn_right_fast()
                    time.sleep(0.01)
                    self.move_stop()
            elif not self.io_data[io_front] and self.adc_data[adc_front] > adc_threshold[adc_front]:
                print(1)
                self.move_stop()
                self.turn_right_fast()
                time.sleep(t_turn_180)
                self.move_stop()
            elif not self.io_data[io_left] and self.adc_data[adc_left] > adc_threshold[adc_left]:
                print(2)
                self.move_stop()
                self.turn_right_fast()
                time.sleep(t_turn_90)
                self.move_stop()
            elif not self.io_data[io_right] and self.adc_data[adc_right] > adc_threshold[adc_right]:
                print(3)
                self.move_stop()
                self.turn_left_fast()
                time.sleep(t_turn_90)
                self.move_stop()
            elif not self.io_data[io_back] and self.adc_data[adc_back_left] > adc_threshold[adc_back_left] \
                and self.adc_data[adc_back_right] <= self.adc_data[adc_back_left] * 0.45:
                print(4)
                self.move_stop()
                self.turn_left_fast()
                time.sleep(t_turn_45)
                self.move_up()
                time.sleep(t_a_little_up)
                self.turn_right_fast()
                time.sleep(t_turn_45)
                self.move_stop()
            elif not self.io_data[io_back] and self.adc_data[adc_back_right] > adc_threshold[adc_back_right] \
                and self.adc_data[adc_back_left] <= self.adc_data[adc_back_right] * 0.55:
                print(5)
                self.move_stop()
                self.turn_right_fast()
                time.sleep(t_turn_45)
                self.move_up()
                time.sleep(t_a_little_up)
                self.turn_left_fast()
                time.sleep(t_turn_45)
                self.move_stop()
            elif not self.io_data[io_front] and self.io_data[io_back] and self.io_data[io_left] and self.io_data[io_right]:
                print(6)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and not self.io_data[io_back] and self.io_data[io_left] and self.io_data[io_right]:
                print(7) 
                self.move_stop()
                self.move_back()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and self.io_data[io_back] and not self.io_data[io_left] and self.io_data[io_right]:
                print(8)
                self.move_stop()
                self.turn_left_fast()
                time.sleep(t_turn_90)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and self.io_data[io_back] and self.io_data[io_left] and not self.io_data[io_right]:
                print(9)
                self.move_stop()
                self.turn_right_fast()
                time.sleep(t_turn_90)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and not self.io_data[io_back] and self.io_data[io_left] and not self.io_data[io_right]:
                print(10)
                self.move_stop()
                self.move_back()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and not self.io_data[io_back] and not self.io_data[io_left] and self.io_data[io_right]:
                print(11)
                self.move_stop()
                self.move_back()
                time.sleep(0.6)
                self.move_stop()
            elif not self.io_data[io_front] and self.io_data[io_back] and not self.io_data[io_left] and self.io_data[io_right]:
                print(12)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif not self.io_data[io_front] and self.io_data[io_back] and self.io_data[io_left] and not self.io_data[io_right]:
                print(13)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif self.io_data[io_front] and self.io_data[io_back] and not self.io_data[io_left] and not self.io_data[io_right]:
                print(14)
                self.move_stop()
                if time.time() % 2:
                    self.turn_right_fast()
                else:
                    self.turn_left_fast()
                time.sleep(t_turn_90)
                self.move_stop()
                self.move_up()
                time.sleep(0.6)
                self.move_stop()
            elif not self.io_data[io_front] and not self.io_data[io_back] and self.io_data[io_left] and self.io_data[io_right]:
                print(15)
                self.move_stop()
                if time.time() % 2 :
                    self.move_up()
                else:
                    self.move_back()
                time.sleep(0.6)
                self.move_stop()   
            else:
                print("orz")
                self.move_back()
                time.sleep(0.1)
                self.move_stop() 

    def on_stage(self):
        if not self.io_data[io_back] and (self.adc_data[adc_back_right] > adc_threshold[adc_back_right] or self.adc_data[adc_back_left] > adc_threshold[adc_back_left]):
            return False
        if not self.io_data[io_front] and self.adc_data[adc_front] > adc_threshold[adc_front]:
            return False
        if not self.io_data[io_left] and self.adc_data[adc_left] > adc_threshold[adc_left]:
            return False
        if not self.io_data[io_right] and self.adc_data[adc_right] > adc_threshold[adc_right]:
            return False
        return True

if __name__ == '__main__':
    c = MyController()
    c.move_stop()
    time.sleep(1)
    #c.upstage()
    #c.p_edge_detect.start()
    #c.turn_left_slow()
    c.upstage_adjust()
    

    


        

    



        
  

    
