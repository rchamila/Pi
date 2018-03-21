import threading
import time
import datetime
import RPi.GPIO as GPIO 
import Helper


from IMU import *
from picamera import PiCamera
from Helper import *


log = LogHelper()

log.logInfo('Data capturing started...')
log.logInfo('******************************************************')

GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN)
imu = IMU()

def worker():
    imu.read()

try:
    command = ''
    camera = PiCamera()

    while (True):
        #command = raw_input('Do you want to take a photo y/n ? ')
        if(command == 'y'):
            command = ''
        #if(GPIO.input(17)):
            
            thread = threading.Thread(target=worker)
           
            log.logInfo('Button press detected...')
            log.logInfo('=====================================================')

            imu.READ_IMU_DATA = 1
            
            thread.start()             
            
            #Wait 200 ms to start data capturing, enabling sensors to initialise
            time.sleep(0.2)

            camera.start_preview()

            log.logInfo('Camera started...')

            i = datetime.datetime.now()  
            imagefile = "/home/pi/source/DC/Images/"+ str(i) + ".jpg"
            camera.capture(imagefile)

            log.logInfo('Image captured and saved to '  + imagefile)

            camera.stop_preview()

            log.logInfo('Camera stopped...')
            
            imu.READ_IMU_DATA = 0

            datafile = "/home/pi/source/DC/DataFiles/"+ str(i) + ".txt"
                        
            f= open(datafile,"w+")
            for i in imu.DATA:
                f.write('%s,%s,%s,%s,%s,%s,%s,%s,%s,%s \n' % (str(i[0]),str(i[1]),str(i[2]),str(i[3]),str(i[4]),str(i[5]),str(i[6]),str(i[7]),str(i[8]),str(i[9])))
            f.close()

            log.logInfo("Data captured and saved to  " + datafile)
        else:
            command = raw_input('Do you want to take a photo y/n ? ')
            if command == 'y':
                continue;
            else:
                sys.exit();
except Exception:
    log.logError("Error in capturing data") 


log.logInfo('End of main thread')
log.logInfo('*************************************************')
