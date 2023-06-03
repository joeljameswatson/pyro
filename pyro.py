# Below imports all neccessary packages to make this Python Script run
import time
# import board
from adafruit_motor import stepper
from adafruit_motorkit import MotorKit

# Below initialises the variable kit to be our I2C Connected Adafruit Motor HAT
# kit = MotorKit(i2c=board.I2C()) # doesn't appear to be needed?
kit = MotorKit()
# SINGLE, DOUBLE, INTERLEAVED, MICROSTEP
# 100 steps = low to max

rangeVal = 20

for i in range(rangeVal):
    res = kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.SINGLE)
    time.sleep(0.01)
    print(res)
    
time.sleep(1)

for i in range(rangeVal):
     res = kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.SINGLE)
     time.sleep(0.01)
     print(res)


# The below line will de-energise the Stepper Motor so it can freely move
kit.stepper2.release()
