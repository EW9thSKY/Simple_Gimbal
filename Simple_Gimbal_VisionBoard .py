# Untitled - By: 陈烁 - Mon Sep 30 2024

import sensor, image, time
from pyb import UART
import ustruct

blue_threshold   = ((0, 80, 21, 66, -128, 5)) #阈值有问题

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

#sensor.set_auto_exposure(False, 800)#降低曝光
sensor.set_auto_whitebal(False) # 关闭白平衡.
sensor.set_auto_gain(False) #关闭自动增益
clock = time.clock()

uart = UART(2, 115200)
uart.init(115200, bits=8, parity=None, stop=1)  #8位数据位，无校验位，1位停止位



def FindMaxBlob(blobs):
    max_blob = None
    max_size = 0
    if blobs:
        for blob in blobs:
            if blob.pixels() > max_size:
                max_blob = blob
                max_size = blob.pixels()
    else:
        print("Not finding any blobs!")
    return max_blob


def Blob_DrawTag(img,blob):
    if blob:
        img.draw_circle(blob.cx(),blob.cy(),20)
        img.draw_cross(blob.cx(),blob.cy(),size = 20)



while(True):
    clock.tick()
    img = sensor.snapshot()
    blue_blobs = img.find_blobs([blue_threshold], pixels_threshold=4, area_threshold=8, merge=False)
    blue_Maxblob = FindMaxBlob(blue_blobs)
    if blue_Maxblob:
        Blob_DrawTag(img,blue_Maxblob)
        cx = blue_Maxblob.cx()
        cy = blue_Maxblob.cy()
        # 将两个字节对象拼接在一起
        FH = bytearray([0x2C,0x12,(cx>>8)&0xff,cx&0xff,cy,0x5B])
        # 发送数据
        uart.write(FH)
        print("Sent:", cx, cy)
    print(clock.fps())
