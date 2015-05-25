#!/usr/bin/python
#
# THIS IS THE PI2GO LIBRARY! GREETINGS HUMAN!
#
# Python Module to externalise all Pi2Go specific hardware
#
# Created by Gareth Davies and Zachary Igielman, May 2014
# Updated June 2014 to include Pi2Go-Lite within same framework
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
# Utilised by Matthew Timmons-Brown for officially recognised tutorials - significant modification since original script
#======================================================================
#======================================================================
# General Functions
# (Both versions)
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
# version(). Returns 1 for Full Pi2Go, and 2 for Pi2Go-Lite. Invalid until after init() has been called
#======================================================================
#======================================================================
# Motor Functions
# (Both Versions)
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
# go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100
#======================================================================
#======================================================================
# RGB LED Functions
# (Full Pi2Go only)
#
# setLED(LED, Red, Green, Blue): Sets the LED specified to required RGB value. 0 >= LED <= 4; 0 <= R,G,B <= 4095
# setAllLEDs(Red, Green, Blue): Sets all LEDs to required RGB. 0 <= R,G,B <= 4095
#======================================================================
#======================================================================
# WHITE LED Functions
# (Pi2Go-Lite only)
#
# LsetLED(LED, value): Sets the LED specified to OFF == 0 or ON >= 1
# LsetAllLEDs(value): Sets both LEDs to OFF == 0 or ON >= 1
#======================================================================
#======================================================================
# IR Sensor Functions
# (Both Versions)
#
# irLeft(): Returns state of Left IR Obstacle sensor
# irRight(): Returns state of Right IR Obstacle sensor
# irCentre(): Returns state of Centre IR Obstacle sensor (Full Pi2Go Only)
# irAll(): Returns true if any of the Obstacle sensors are triggered
# irLeftLine(): Returns state of Left IR Line sensor
# irRightLine(): Returns state of Right IR Line sensor
#======================================================================
#======================================================================
# UltraSonic Functions
# (Both Versions)
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#======================================================================
#======================================================================
# Light Sensor Functions
# (Full Pi2Go only)
#
# getLight(Sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3
# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
# getLightBL(). Returns the value 0..1023 for Back-Left light sensor
# getLightBR(). Returns the value 0..1023 for Back-Right light sensor
#======================================================================
#======================================================================
# Servo Functions
# 
# startServos(). Initialises the servo background process
# stop Servos(). terminates the servo background process
# setServo(Servo, Degrees). Sets the servo to position in degrees -90 to +90
#======================================================================
#======================================================================
# Switch Functions
# 
# getSwitch(). Returns the value of the tact switch: True==pressed
#======================================================================
# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os
from extra_tools.Adafruit_PWM_Servo_Driver import PWM
from extra_tools.sgh_PCF8591P import sgh_PCF8591P

GPIO.setwarnings(False) # Pah, who needs runtime errors nowadays?! MTB edit

# Define Type of Pi2Go
PGNone = 0
PGFull = 1
PGLite = 2
PGType = PGNone # Set to None until we find out which during init()

# Pins 24, 26 Left Motor
# Pins 19, 21 Right Motor
L1 = 26
L2 = 24
R1 = 19
R2 = 21

# Define obstacle sensors and line sensors
irFL = 7
irFR = 11
irMID = 15  # this sensor not available on Lite version
lineRight = 13
lineLeft = 12

# Define Colour IDs for the RGB LEDs (Pi2Go full only)
Blue = 0
Green = 1
Red = 2
pwmMax = 4095 # maximum PWM value

# Define GPIO pins for Front/rear LEDs on Pi2Go-Lite
frontLED = 15
rearLED = 16

# Define Sonar Pin (same pin for both Ping and Echo
sonar = 8

# Define pins for switch (different on each version)
switch = 16
Lswitch = 23

# Define if servo background process is active
ServosActive = False

#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
def init():
    global p, q, a, b, pwm, pcfADC, PGType
    PGType = PGFull
    # Initialise the PCA9685 PWM device using the default address
    try:
        pwm = PWM(0x40, debug = False)
        pwm.setPWMFreq(60)  # Set frequency to 60 Hz
    except:
        PGType = PGLite # No PCA9685 so set to Pi2Go-Lite

    #use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #Set up IR obstacle sensors as inputs
    GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
    GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor
    GPIO.setup(irMID, GPIO.IN) # Centre Front obstacle sensor

    #use pwm on inputs so motors don't go too fast
    GPIO.setup(L1, GPIO.OUT)
    p = GPIO.PWM(L1, 20)
    p.start(0)

    GPIO.setup(L2, GPIO.OUT)
    q = GPIO.PWM(L2, 20)
    q.start(0)

    GPIO.setup(R1, GPIO.OUT)
    a = GPIO.PWM(R1, 20)
    a.start(0)

    GPIO.setup(R2, GPIO.OUT)
    b = GPIO.PWM(R2, 20)
    b.start(0)

    # Initalise the ADC
    pcfADC = None # ADC object
    try:
        pcfADC = sgh_PCF8591P(1) #i2c, 0x48)
    except:
        PGType = PGLite

    # initialise servos (Pi2Go-Lite only) Matt TB - Servo activation not necessary on init
    #if PGType == PGLite:
    #    startServos()

    #set up Pi2Go-Lite White LEDs as outputs
        GPIO.setup(frontLED, GPIO.OUT)
        GPIO.setup(rearLED, GPIO.OUT)

    #set switch as input with pullup
    if PGType == PGLite:
        GPIO.setup(Lswitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    else:
        GPIO.setup(switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
def cleanup():
    stop()
    setAllLEDs(0, 0, 0)
    stopServod()
    time.sleep(1)
    GPIO.cleanup()


# version(). Returns 1 for Full Pi2Go, and 2 for Pi2Go-Lite. Invalid until after init() has been called
def version():
    return PGType

# End of General Functions
#======================================================================


#======================================================================
# Motor Functions
# (both versions)
#
# stop(): Stops both motors
def stop():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)
    
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)
    
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    q.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    q.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)
    
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    p.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)
    
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(leftSpeed + 5)
    a.ChangeFrequency(rightSpeed + 5)
    
# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    q.ChangeFrequency(leftSpeed + 5)
    b.ChangeFrequency(rightSpeed + 5)

# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
def go(leftSpeed, rightSpeed):
    if leftSpeed<0:
        p.ChangeDutyCycle(0)
        q.ChangeDutyCycle(abs(leftSpeed))
        q.ChangeFrequency(abs(leftSpeed) + 5)
    else:
        q.ChangeDutyCycle(0)
        p.ChangeDutyCycle(leftSpeed)
        p.ChangeFrequency(leftSpeed + 5)
    if rightSpeed<0:
        a.ChangeDutyCycle(0)
        b.ChangeDutyCycle(abs(rightSpeed))
        p.ChangeFrequency(abs(rightSpeed) + 5)
    else:
        b.ChangeDutyCycle(0)
        a.ChangeDutyCycle(rightSpeed)
        p.ChangeFrequency(rightSpeed + 5)

# go(speed): controls motors in both directions together with positive/negative speed parameter. -100<= speed <= 100
def goBoth(speed):
    if speed<0:
        reverse(abs(speed))
    else:
        forward(speed)

# End of Motor Functions
#======================================================================


#======================================================================
# RGB LED Functions
# (Full version only)
#
# setLED(LED, Red, Green, Blue): Sets the LED specified to required RGB value. 0 >= LED <= 3; 0 <= R,G,B <= 4095
def setLED(LED, red, green, blue):
    if PGType == PGFull:
        pwm.setPWM(LED * 3 + Red, 0, red)
        pwm.setPWM(LED * 3 + Green, 0, green)
        pwm.setPWM(LED * 3 + Blue, 0, blue)

# setAllLEDs(Red, Green, Blue): Sets all LEDs to required RGB. 0 <= R,G,B <= 4095
def setAllLEDs (red, green, blue):
  for i in range(4):
    setLED(i, red, green, blue)

# End of RGB LED Functions
#======================================================================


#======================================================================
# White LED Functions
# (Pi2Go-Lite only)
#
# LsetLED(LED, value): Sets the LED specified to OFF == 0 or ON == 1
# TODO: take value from 0 to 100 and use as percentage PWM value
def LsetLED (LED, value):
    if PGType == PGLite:
        if value == 0:
            value = 1
        else:
            value = 0
        if LED == 0:
            GPIO.output (frontLED, value)
        else:
            GPIO.output (rearLED, value)
        
# LsetAllLEDs(value): Sets both LEDs to OFF == 0 or ON == 1

# End of White LED Functions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
def irLeft():
    if GPIO.input(irFL)==0:
        return True
    else:
        return False
    
# irRight(): Returns state of Right IR Obstacle sensor
def irRight():
    if GPIO.input(irFR)==0:
        return True
    else:
        return False
    
# irCentre(): Returns state of Centre IR Obstacle sensor
# (Not available on Pi2Go-Lite)
def irCentre():
    if PGType != PGFull:
        return False
    if GPIO.input(irMID)==0:
        return True
    else:
        return False
    
# irAll(): Returns true if any of the Obstacle sensors are triggered
def irAll():
    if GPIO.input(irFL)==0 or GPIO.input(irFR)==0 or (PGType==PGFull and GPIO.input(irMID)==0):
        return True
    else:
        return False
    
# irLeftLine(): Returns state of Left IR Line sensor
def irLeftLine():
    if GPIO.input(lineLeft)==0:
        return True
    else:
        return False
    
# irRightLine(): Returns state of Right IR Line sensor
def irRightLine():
    if GPIO.input(lineRight)==0:
        return True
    else:
        return False
    
# End of IR Sensor Functions
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
# (Both versions)
#
def getDistance():
    GPIO.setup(sonar, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar, True)
    time.sleep(0.00001)
    GPIO.output(sonar, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar,GPIO.IN)
    while GPIO.input(sonar)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

# End of UltraSonic Functions    
#======================================================================


#======================================================================
# Light Sensor Functions
# (Full Pi2Go Only)
#
# getLight(sensor). Returns the value 0..1023 for the selected sensor, 0 <= Sensor <= 3
def getLight(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(sensor)
    return value

# getLightFL(). Returns the value 0..1023 for Front-Left light sensor
def getLightFL():
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(0)
    return value

# getLightFR(). Returns the value 0..1023 for Front-Right light sensor
def getLightFR(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(1)
    return value

# getLightBL(). Returns the value 0..1023 for Back-Left light sensor
def getLightBL(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(2)
    return value

# getLightBR(). Returns the value 0..1023 for Back-Right light sensor
def getLightBR(sensor):
    if PGType != PGFull:
        return False
    value  = pcfADC.readADC(3)
    return value

# End of Light Sensor Functions
#======================================================================


#======================================================================
# Switch Functions
# 
# getSwitch(). Returns the value of the tact switch: True==pressed
def getSwitch():
    if PGType == 1:
        val = GPIO.input(switch)
    else:
        val = GPIO.input(Lswitch)
    return (val == 0)
#
# End of switch functions
#======================================================================



#======================================================================
# Servo Functions
# Pi2Go-Lite uses ServoD to control servos
# Pi2Go Full uses the PCA9685 hardware controller

def setServo(Servo, Degrees):
    #print "ServosActive:", ServosActive
    if ServosActive == False:
        startServos()
    pinServod (Servo, Degrees) # for now, simply pass on the input values

def stopServos():
    stopServod()
    
def startServos():
    startServod()
    
def startServod():
    global ServosActive
    #print "Starting servod. ServosActove:", ServosActive
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    #os.system("sudo pkill -f servod")
    os.system(SCRIPTPATH +'/servod --idle-timeout=20000 --p1pins="18,22"')
    #print (SCRIPTPATH +'/servod --idle-timeout=20000 --p1pins="18,22"')
    ServosActive = True

def pinServod(pin, degrees):
    #print pin, degrees
    #print ("echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster")
    os.system("echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster")
    
def stopServod():
    global ServosActive
    os.system("sudo pkill -f servod")
    ServosActive = False
        


