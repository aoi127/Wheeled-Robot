import sys
import cv2
import apriltag
import numpy as np
from my_controller import MyController
from multiprocessing import Process, Manager, Value, Array
import time


if __name__ == '__main__':
    #time.sleep(20)
    c = MyController()
    c.move_stop()
    time.sleep(1)

    c.upstage()
    #c.p_edge_detect.start()
    while True:
        # print('c.enemy_block_is_down.value : ', c.enemy_block_is_down.value)
        if not c.move_flag.value:
            if c.find_by_vision():
                c.attack()
                pass

            
    


