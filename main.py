import threading
import time
import datetime
from IMU import *
from picamera import PiCamera


imu = IMU();

def worker():
    imu.read();
    I =0;



thread = threading.Thread(target=worker)

try:
    
    camera = PiCamera();
    i = datetime.datetime.now();

    imu.READ_IMU_DATA = 1;
    thread.start();
    camera.start_preview();
    camera.capture('/home/pi/source/DC/'+ str(i) + '.jpg')
    camera.stop_preview()
    imu.READ_IMU_DATA = 0;
    # thread.cancel();
    # sys.exit();
except(KeyboardInterrupt, SystemExit):
    thread.cancel();
    # sys.exit();
# imu = IMUTest();
# imu.read();
print('End of current thread')
