import threading
import time
import datetime
import RPi.GPIO as GPIO



from IMU import *
from picamera import PiCamera

print('Data capturing started...')

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)
imu = IMU()

def worker():
    imu.read()

try:
    #command = ''
    camera = PiCamera()
    while (True):
        #if(command == 'y'):
            #command = ''
        if(GPIO.input(17)):
            
            thread = threading.Thread(target=worker)

            i = datetime.datetime.now()

            imu.READ_IMU_DATA = 1
            
            thread.start()
            
            camera.start_preview()
            
            camera.capture("/home/pi/source/DC/Images/"+ str(i) + ".jpg")
            camera.stop_preview()
            
            imu.READ_IMU_DATA = 0
            
            f= open("/home/pi/source/DC/DataFiles"+ str(i) + ".txt","w+")
            for i in imu.DATA:
                f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s \n' % (str(i[0]),str(i[1]),str(i[2]),str(i[3]),str(i[4]),str(i[5]),str(i[6]),str(i[7]),str(i[8]),str(i[9])))
                #f.write('%s,%s,%s \n' % (str(i[0]),str(i[1]),str(i[2])))
            f.close()
            #thread.cancel()
        #else:
            #command = raw_input('Do you want to take a photo y/n ? ')
    # sys.exit();
except(KeyboardInterrupt, SystemExit):
    thread.cancel();
    # sys.exit();
print('End of current thread')
