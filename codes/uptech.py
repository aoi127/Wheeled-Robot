#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pigpio
import time
import threading
# from ctypes import *
# import numpy as np
import binascii
from ctypes import cdll
#import numpy as np
import ctypes
__version__="1.0"
# hPi=pigpio.pi()
# #hPi=pigpio.pi()
# hSpi0=-1
# hSpi1=-1

FAN_GPIO_PWM=18



#so_up = cdll.LoadLibrary("/home/pi/Desktop/backup_a_lib/pytlib/libuptech.so")
so_up = cdll.LoadLibrary("libuptech.so")
def SWAP(x,y):
    temp=x
    x=y
    y=temp



class UpTech:

    CDS_MODE_SERVO = 0  #舵机模式
    CDS_MODE_MOTOR = 1  #电机模式
##字体大小，依次递增
    FONT_4X6    = 0
    FONT_5X8    = 1
    FONT_5X12   = 2
    FONT_6X8    = 3
    FONT_6X10   = 4
    FONT_7X12   = 5
    FONT_8X8    = 6
    FONT_8X12   = 7
    FONT_8X14   = 8
    FONT_10X16  = 9
    FONT_12X16  = 10
    FONT_12X20  = 11
    FONT_16X26  = 12
    FONT_22X36  = 13
    FONT_24X40  = 14
##颜色表
    COLOR_WHITE         	 =0xFFFF
    COLOR_BLACK         	 =0x0000	  
    COLOR_BLUE           	 =0x001F  
    COLOR_BRED               =0XF81F
    COLOR_GRED 			     =0XFFE0
    COLOR_GBLUE			     =0X07FF
    COLOR_RED           	 =0xF800
    COLOR_MAGENTA       	 =0xF81F
    COLOR_GREEN         	 =0x07E0
    COLOR_CYAN          	 =0x7FFF
    COLOR_YELLOW        	 =0xFFE0
    COLOR_BROWN 			 =0XBC40
    COLOR_BRRED 			 =0XFC07 
    COLOR_GRAY  			 =0X8430 
    COLOR_DARKBLUE      	 =0X01CF	
    COLOR_LIGHTBLUE      	 =0X7D7C
    COLOR_GRAYBLUE       	 =0X5458 
    COLOR_LIGHTGREEN     	 =0X841F 
    COLOR_LGRAY 			 =0XC618 
    COLOR_LGRAYBLUE          =0XA651
    COLOR_LBBLUE             =0X2B12 
###adc
    __adc_data = ctypes.c_uint16*10
    __ADC_DATA=__adc_data()
    ADC_DATA = [0]*10
##imu
    __mpu_float =  ctypes.c_float*3
    __MPU_DATA = __mpu_float()
    ACCEL_DATA = [0]*3
    GYRO_DATA = [0]*3
    ATTITUDE = [0]*3
##初始化
    def __init__(self):

        pigpio.exceptions = False
        self.hPi = pigpio.pi()
        if not self.hPi.connected :
            exit

        # self.hSpi1 = self.hPi.spi_open(2, 1000000, (1<<8)|(0<<0))
        # if self.hSpi1 < 0:
        #     self.hPi.spi_close(1)
        #     self.hSpi1 = self.hPi.spi_open(2, 1000000, (1<<8)|(0<<0))
        pigpio.exceptions = True
        self.hPi.hardware_PWM(FAN_GPIO_PWM,20000,1000000)
        self.hPi.set_PWM_range(FAN_GPIO_PWM,100)


    # def __LCD_INTTERUPT(self):

    #     adc_value=self.ADC_Get_All_Channle()
    #     print adc_value
    #     if self.__lcd_timer_runnig:
    #         self.__lcd_refresh_timer=threading.Timer(0.01,self.__LCD_INTTERUPT)
    #         self.__lcd_refresh_timer.start()
    #     else:
    #         self.__lcd_refresh_timer.cancel()

    def stop(self):
        # # self.timer_LCD()
        # self.__lcd_timer_runnig=False
        # time.sleep(0.1)
        # #self.__lcd_refresh_timer

        # self.hPi.spi_close(self.hSpi1)
        # self.hPi.stop()
        pass

    def FAN_Set_Speed(self,speed):
        if self.hPi >= 0:
            if speed>100:
                speed=100
            elif speed<0:
                speed=0
            self.hPi.set_PWM_dutycycle(FAN_GPIO_PWM,speed)

    def ADC_IO_Open(self):          #打开adc io
        return so_up.adc_io_open()
    def ADC_IO_Close(self):         #关闭adc io
        so_up.adc_io_close()
    def ADC_Get_All_Channle(self): #获取adc端口值大小
        # adc = so_up.ADC_GetAll
        # adc.argtypes = [np.ctypeslib.ndpointer(dtype=np.int16,ndim=1,flags="C_CONTIGUOUS")]
        # res = np.zeros(10,dtype=np.int16)*0
        # adc(res)
        # print res

        so_up.ADC_GetAll(self.__ADC_DATA)
        for i in range(10):
            self.ADC_DATA[i] = self.__ADC_DATA[i]
        # print self.ADC_DATA
        return self.ADC_DATA


        # so_up.ADC_GetAll(self.__ADC_DATA)
        # print self.__ADC_DATA.tolist()
##设置led颜色
    def ADC_Led_SetColor(self,index,RGB):
        so_up.adc_led_set(index,RGB)

    ##设置某个io电平
    def ADC_IO_SetIOLevel(self,index,level):
        so_up.adc_io_Set(index,level)

    ##设置8个io电平
    def ADC_IO_SetAllIOLevel(self,value):
        so_up.adc_io_SetAll(value)        
##设置8个io口模式
    def ADC_IO_SetAllIOMode(self,mode):
        so_up.adc_io_ModeSetAll(mode)
##设置某个io口模式，0输入，1输出
    def ADC_IO_SetIOMode(self,index,mode):
        so_up.adc_io_ModeSet(index,mode)
##获取所有io口电平状态
    def ADC_IO_GetAllInputLevel(self):
        return so_up.adc_io_InputGetAll()
##初始化舵机
    def CDS_Open(self):
        so_up.cds_servo_open()
##关闭舵机模块
    def CDS_Close(self):
        so_up.cds_servo_close()
##设置舵机模式，0舵机，1电机，断电有效
    def CDS_SetMode(self,id,mode):
        so_up.cds_servo_SetMode(id,mode)
##设置舵机角度，angle：0-1023,speed：0-1023
    def CDS_SetAngle(self,id,angle,speed):
        so_up.cds_servo_SetAngle(id,angle,speed)
##设置舵机速度，-1023-1023
    def CDS_SetSpeed(self,id,speed):
        so_up.cds_servo_SetSpeed(id,speed)
##获取当前舵机位置
    def CDS_GetCurPos(self,id):
        return so_up.cds_servo_GetPos(id)
##初始化mpu6500
    def MPU6500_Open(self):
        so_up.mpu6500_dmp_init()
 ##获取加速度
    def MPU6500_GetAccel(self):
        so_up.mpu6500_Get_Accel(self.__MPU_DATA)
        for i in range(3):
            self.ACCEL_DATA[i] = self.__MPU_DATA[i]
        return self.ACCEL_DATA
##获取角速度
    def MPU6500_GetGyro(self):
        so_up.mpu6500_Get_Gyro(self.__MPU_DATA)
        for i in range(3):
            self.GYRO_DATA[i] = self.__MPU_DATA[i]
        return self.GYRO_DATA
##获取姿态数据，pitch roll yaw
    def MPU6500_GetAttitude(self):
        so_up.mpu6500_Get_Attitude(self.__MPU_DATA)
        for i in range(3):
            self.ATTITUDE[i] = self.__MPU_DATA[i]
        return self.ATTITUDE
##初始化lcd
    def LCD_Open(self,dir):
        return so_up.lcd_open(dir)

    def LCD_Refresh(self):
        so_up.LCD_Refresh()
#size of word
    def LCD_SetFont(self,font_index):  
        so_up.LCD_SetFont(font_index)
##字体颜色
    def LCD_SetForeColor(self,color):
        so_up.UG_SetForecolor(color)
##字体背景颜色
    def LCD_SetBackColor(self,color):
        so_up.UG_SetBackcolor(color)
 ##lcd背景颜色
    def LCD_FillScreen(self,color):
        so_up.UG_FillScreen(color)
##lcd打印
    def LCD_PutString(self,x,y,str):
        byte=ctypes.c_byte*len(str)
        bin=byte()
        i=0
        for c in str:
            bin[i] = ord(c)
            i += 1
        so_up.UG_PutString(x,y,bin)
##lcd区域颜色
    def LCD_FillFrame(self,x1,y1,x2,y2,color):
        so_up.UG_FillFrame(x1,y1,x2,y2,color)
##
    def LCD_FillRoundFrame(self,x1,y1,x2,y2,r,color):
        so_up.UG_FillRoundFrame(x1,y1,x2,y2,r,color)

    def LCD_DrawMesh(self,x1,y1,x2,y2,color):
        so_up.UG_DrawMesh(x1,y1,x2,y2,color)
    
    def LCD_DrawFrame(self,x1,y1,x2,y2,color):
        so_up.UG_DrawFrame(x1,y1,x2,y2,color)

    def LCD_DrawRoundFrame(self,x1,y1,x2,y2,r,color):
        so_up.UG_DrawRoundFrame(x1,y1,x2,y2,r,color)

    def LCD_DrawPixel(self,x0,y0,color):
        so_up.UG_DrawPixel(x0,y0,color)

    def LCD_DrawCircle(self,x0,y0,r,color):
        so_up.UG_DrawCircle(x0,y0,r,color)

    def LCD_FillCircle(self,x0,y0,r,color):
        so_up.UG_FillCircle(x0,y0,r,color)

    def LCD_DrawArc(self,x0,y0,r,s,color):
        so_up.UG_DrawArc(x0,y0,r,s,color)

    def LCD_DrawLine(self,x1,y1,x2,y2,color):
        so_up.UG_DrawLine(x1,y1,x2,y2,color)        
    # void UG_DrawArc( UG_S16 x0, UG_S16 y0, UG_S16 r, UG_U8 s, UG_COLOR c );
    # void UG_DrawLine( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c );
if __name__ == "__main__":
    move_up()
    pass
