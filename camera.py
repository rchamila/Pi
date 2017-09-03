from picamera import PiCamera
from time import sleep
from gpiozero import Button

Button = Button(17)
camera = PiCamera()

camera.start_preview()
#Button.wait_for_press()
#sleep(3)
camera.capture('/home/pi/source/DC/image4.jpg')
camera.stop_preview()
