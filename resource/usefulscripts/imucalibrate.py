import time
import board
import busio
import adafruit_mpu6050
 
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)
 
 
meanax = 0
meanay = 0
meanaz = 0
meangx = 0
meangy = 0
meangz = 0
 
 
for i in range(5000):
   
    meanax += mpu.acceleration[0]
    meanay += mpu.acceleration[1]
    meanaz += mpu.acceleration[2]
    meangx += mpu.gyro[0]
    meangy += mpu.gyro[1]
    meangz += mpu.gyro[2]
 
       
    print("------iteration-------",i+1)
    time.sleep(1)
 
print("Acceleration: X:%.4f, Y: %.4f, Z: %.4f m/s^2" % (meanax/5000,meanay/5000,meanaz/5000))
print("Gyro X:%.4f, Y: %.4f, Z: %.4f degrees/s" % (meangx/5000,meangy/5000,meangz/5000))