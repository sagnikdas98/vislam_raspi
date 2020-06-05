import os
import pathlib
from time import sleep
from picamera import PiCamera



camera = PiCamera()
camera.resolution = (640,640)
camera.vflip = True
sleep(2)
N = 40
relative_package_path = ""

def funcsleep(i):
    print(1)
    sleep(1)
    print(2)
    sleep(1)
    print(3)
    sleep(1)
    print(4)
    sleep(1)
    print("-------------Capturing:",i,"-------------")
    

for i in range(1,N+1):
    output_file_name = relative_package_path + "calibrateimages/image{}.jpg".format(i)
    camera.start_preview()
    funcsleep(i)
    camera.capture(output_file_name)