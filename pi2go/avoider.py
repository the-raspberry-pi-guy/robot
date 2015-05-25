# Pi2Go 'avoider sketch' - for the second episode of my robot tutorial series
# This program is fairly simple - it utilises the IR and ultrasonic sensors
# on the Pi2Go in order to sense obstacles and avoid them
# Created by Matthew Timmons-Brown and Simon Beal

import pi2go, time

pi2go.init()

# Here we set the speed to 40 out of 100 - feel free to change!
speed = 40

# Here is the main body of the program - a lot of while loops and ifs!
# In order to get your head around it go through the logical steps slowly!
try:
  while True:
    if pi2go.irLeft():
      while pi2go.irLeft():
        # While the left sensor detects something - spin right
        pi2go.spinRight(speed)
      pi2go.stop()
    if pi2go.irRight():
      while pi2go.irRight():
        # While the right sensor detects something - spin left
        pi2go.spinLeft(speed)
      pi2go.stop()
    while not (pi2go.irLeft() or pi2go.irRight()):
      if pi2go.getDistance() <= 0.3: # If the distance is less than 0.3, spin right for 1 second
        pi2go.spinRight(speed)
        time.sleep(1)
      else:
        pi2go.forward(speed)
    pi2go.stop()

finally: # Even if there was an error, cleanup
  pi2go.cleanup()
