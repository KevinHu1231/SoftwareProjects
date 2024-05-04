# import RPi.GPIO as GPIO
# import time
# 
# GPIO.setmode(GPIO.BOARD)
# 
# GPIO.setup(11, GPIO.OUT)
# servo1 = GPIO.PWM(11, 50)
# 
# servo1.start(0)
# 
# 
# 
# try:
#     while True:
#         angle = float(input('Enter angle between 0 and 180: '))
#         servo1.ChangeDutyCycle(12)
#         time.sleep(0.5)
#         servo1.ChangeDutyCycle(0)
#         
# finally:
#     servo1.stop()
#     GPIO.cleanup()
#     print('Program stopped')



# print ("Waiting for 2 seconds")
# time.sleep(2)
# 
# #Let's move the servo!
# print ("Rotating 180 degrees in 10 steps")
# 
# # Define variable duty
# duty = 2
# 
# # Loop for duty values from 2 to 12 (0 to 180 degrees)
# while duty <= 12:
#     servo1.ChangeDutyCycle(duty)
#     time.sleep(0.3)
#     servo1.ChangeDutyCycle(0)
#     time.sleep(0.7)
#     duty = duty + 1
# 
# # Wait a couple of seconds
# time.sleep(2)
# 
# # Turn back to 90 degrees
# print ("Turning back to 90 degrees for 2 seconds")
# servo1.ChangeDutyCycle(7)
# time.sleep(0.5)
# servo1.ChangeDutyCycle(0)
# time.sleep(1.5)
# 
# #turn back to 0 degrees
# print ("Turning back to 0 degrees")
# servo1.ChangeDutyCycle(2)
# time.sleep(0.5)
# servo1.ChangeDutyCycle(0)
# 
# #Clean things up at the end
# servo1.stop()
# GPIO.cleanup()
# print ("Goodbye")

from gpiozero import Servo
import math
from time import sleep
import pigpio

from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo = Servo(17, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)

# print('Starting in the middle')
servo.mid()
sleep(2)
# print('Going to min')
servo.min()
sleep(2)
# print('Going to max')
servo.max()
sleep(2)
# print('Going to middle')
servo.mid()
sleep(2)
servo.value = None
try:
    while True:
        for i in range(0, 360):
            servo.value = math.sin(math.radians(i))
            sleep(0.01)
            print(servo.value)
            print(i)
finally:
    servo.value = 0