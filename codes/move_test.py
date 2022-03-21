import uptech
import time

if __name__ == '__main__': 
    up = uptech.UpTech()
    up.CDS_Open()
    up.CDS_SetMode(1, 1)
    up.CDS_SetMode(2, 1)
    time.sleep(1)
    '''
    up.CDS_SetSpeed(1, 250)
    up.CDS_SetSpeed(2, -250)
    '''
    #time.sleep(1)
    #up.CDS_SetSpeed(1, -1023)
    #up.CDS_SetSpeed(2,1023)
    #time.sleep(1)
    up.CDS_SetSpeed(1,1023)
    print('upstage')
    up.CDS_SetSpeed(2,0)
    time.sleep(0.55)
    up.CDS_SetSpeed(1,0)
    up.CDS_SetSpeed(2,0)
    #time.sleep(1)
