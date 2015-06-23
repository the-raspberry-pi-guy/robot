# Pi2Go 'follower sketch' - for the third episode of my robot tutorial series
# This program is also fairly simple - it utilises the Line IRs
# on the Pi2Go in order to sense obstacles and avoid them
# Created by Matthew Timmons-Brown and Simon Beal

import pi2go, time

pi2go.init()

# Here we set the speed to 60 out of 100 - feel free to change!
speed = 60

try:
  while True:
    # Defining the sensors on the bottom of the Pi2Go
    left = pi2go.irLeftLine()
    right = pi2go.irRightLine()
    if left == right: # If both sensors are the same (either on or off):
      # Forward
      pi2go.forward(speed)
    elif left == True: # If the left sensor is on
      # Left
      pi2go.spinRight(speed)
    elif right == True: #If the right sensor is on
      # Right
      pi2go.spinLeft(speed)

finally: # Even if there was an error, cleanup
  pi2go.cleanup()
