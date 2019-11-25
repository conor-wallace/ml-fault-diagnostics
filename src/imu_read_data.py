import logging
import sys
import subprocess
import time
import json
import matplotlib.pyplot as plt
import numpy as np
import math
import rospy
from network_faults.msg import IMU
from Adafruit_BNO055 import BNO055

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
#<<<<<<< Updated upstream
# output = subprocess.Popen(['ls', r'/dev/ttyUSB*'],stdout = subprocess.PIPE).communicate()[0]
# bno = BNO055.BNO055(serial_port='/dev/ttyUSB0', rst=18)
# =======
bno = BNO055.BNO055(serial_port='/dev/ttyUSB0', rst=17)
# >>>>>>> Stashed changes
bno.begin()
CALIBRATION_FILE = './../data/calibration.json'
first_read = 1
offset = None

# Load calibration from disk.
with open(CALIBRATION_FILE, 'r') as cal_file:
    data = json.load(cal_file)
    # Grab the lock on BNO sensor access to serial access to the sensor.
    bno.set_calibration(data)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

print('Reading BNO055 data, press Ctrl-C to quit...')
rospy.init_node('read_imu')
imu_pub = rospy.Publisher('/imu', IMU, queue_size=1, tcp_nodelay=True)
imu_msg = IMU()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    sys, gyro, accel, mag = bno.get_calibration_status()
    heading, roll, pitch = bno.read_euler()
    accel_x,accel_y,accel_z = bno.read_accelerometer()
    lin_x,lin_y,lin_z = bno.read_linear_acceleration()
# <<<<<<< Updated upstream
    data.append([heading, roll, pitch, accel_x, accel_y, accel_z, lin_x, lin_y, lin_z])
    print("sys: %s, gyro: %s, accel: %s, mag: %s" % (sys,gyro,accel,mag))
    if first_read:
        offset.append([heading, roll, pitch, accel_x, accel_y, accel_z, lin_x, lin_y, lin_z])
# =======
    data = [heading, roll, pitch, accel_x, accel_y, accel_z, lin_x, lin_y, lin_z]
    print("sys: %s, gyro: %s, accel: %s, mag: %s" % (sys,gyro,accel,mag))
    if first_read:
        offset = data
# >>>>>>> Stashed changes
        first_read = 0
    else:
	print(len(data))
	print(len(offset))
        for i in range(len(data)):
            data[i] = data[i] - offset[i]
    imu_msg.yaw = data[0] % 360
    imu_msg.pitch = data[1]
    imu_msg.roll = data[2]
    imu_msg.accel_x = data[3]
    imu_msg.accel_y = data[4]
    imu_msg.accel_z = data[5]
    imu_msg.lin_x = data[6]
    imu_msg.lin_y = data[7]
    imu_msg.lin_z = data[8]

    imu_pub.publish(imu_msg)
    rate.sleep()
