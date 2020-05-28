from MPU6050 import MPU6050
from SimplePID import SimplePID




class Calib:

    def __init__(self):
        i2c_bus = 1
        device_address = 0x68

        x_accel_offset = 0
        y_accel_offset = 0
        z_accel_offset =0
        x_gyro_offset = 0
        y_gyro_offset = 0
        z_gyro_offset = 0
        enable_debug_output = False

        kp = 0.03125
        ki = 0.25
        kd = 0

        self.n = 200

        self.mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
                    z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
                    enable_debug_output)

        

        self.pidax = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
        self.piday = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
        self.pidaz = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
        self.pidgx = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
        self.pidgy = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
        self.pidgz = SimplePID(0, -15000, 15000, kp, ki, kd, 100, True)
    

    def avg_from_array(a_array):
        return sum(a_array)/len(a_array)


    def imu_calib(self):
        n = self.n

        axindex = 0
        ayindex = 0
        azindex = 0
        gxindex = 0
        gyindex = 0
        gzindex = 0


        x_accel_offset_avg = [0]*n
        y_accel_offset_avg = [0]*n
        z_accel_offset_avg = [0]*n

        x_gyro_offset_avg = [0]*n
        y_gyro_offset_avg = [0]*n
        z_gyro_offset_avg = [0]*n

       

        for i in range(n):

            accel_reading = self.mpu.get_acceleration()
            x_accel_reading = accel_reading[0]
            y_accel_reading = accel_reading[1]
            z_accel_reading = accel_reading[2]

            gyro_reading = self.mpu.get_rotation()
            x_gyro_reading = gyro_reading[0]
            y_gyro_reading = gyro_reading[1]
            z_gyro_reading = gyro_reading[2]

            if self.pidax.check_time():
                x_accel_offset = self.pidax.get_output_value(x_accel_reading)
                self.mpu.set_x_accel_offset(int(x_accel_offset))
                x_accel_offset_avg[axindex] = x_accel_offset
                axindex += 1
                        

            if self.piday.check_time():
                y_accel_offset = self.piday.get_output_value(y_accel_reading)
                self.mpu.set_y_accel_offset(int(y_accel_offset))
                y_accel_offset_avg[ayindex] = y_accel_offset
                ayindex += 1
            
                

            if self.pidaz.check_time():
                z_accel_offset = self.pidaz.get_output_value(z_accel_reading)
                self.mpu.set_z_accel_offset(int(z_accel_offset))
                z_accel_offset_avg[azindex] = z_accel_offset
                azindex += 1
        

            if self.pidgx.check_time():
                x_gyro_offset = self.pidgx.get_output_value(x_gyro_reading)
                self.mpu.set_x_gyro_offset(int(x_gyro_offset)) 
                x_gyro_offset_avg[gxindex] = x_gyro_offset
                gxindex+=1



            if self.pidgy.check_time():
                y_gyro_offset = self.pidgy.get_output_value(y_gyro_reading)
                self.mpu.set_y_gyro_offset(int(y_gyro_offset))
                y_gyro_offset_avg[gyindex] = y_gyro_offset
                gyindex += 1

            if self.pidgz.check_time():
                z_gyro_offset = self.pidgz.get_output_value(z_gyro_reading)
                self.mpu.set_z_gyro_offset(int(z_gyro_offset))
                z_gyro_offset_avg[gzindex] = z_gyro_offset
                gzindex += 1


        xaccel = Calib.avg_from_array(x_accel_offset_avg)
        yaccel = Calib.avg_from_array(y_accel_offset_avg)
        zaccel = Calib.avg_from_array(z_accel_offset_avg)
        xgyro = Calib.avg_from_array(x_gyro_offset_avg)
        ygyro = Calib.avg_from_array(y_gyro_offset_avg)
        zgyro = Calib.avg_from_array(z_gyro_offset_avg))


        return(xaccel,yaccel,zaccel,xgyro,ygyro,zgyro)


def main():
    cal = Calib()
    xaloff, yaloff, zaloff, xgyoff, ygyoff, zgyoff = cal.imu_calib()
    print("xaloff, yaloff, zaloff, xgyoff, ygyoff, zgyoff")
    print(xaloff, yaloff, zaloff, xgyoff, ygyoff, zgyoff)


if __name__ == "__main__":
    main()
    


