# Wall-E
A self-balancing robot for an MPCA assignment.

<br />

## Dependencies
The project requires the following libraries :
  - I2Cdev : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
  - MPU6050 : https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
  - NewPing : https://bitbucket.org/teckel12/arduino-new-ping/src/master/

#### NOTE :
This project uses the PlatformIO IDE to build and upload code, hence the above libraries are located in the `lib/` folder.
If you are using the Arduino IDE, copy the source folders of each of the above libraries into your project root, like so :
```
.
└── Wall-E/
    ├── I2Cdev/
    ├── MPU6050/
    ├── NewPing/
    └── Wall-E.ino
```
You can then copy and paste code from the `src/main.cpp` file into `Wall-E.ino`, excluding the first `#include <Arduino.h>` line.

<br />

## Calibration
### MPU6050
The MPU6050 needs to be calibrated and the offsets need to be known. Grab the calibration code from https://wired.chillibasket.com/2015/01/calibrating-mpu6050/ and run it on your Arduino.
Make a note of the `acelY`, `acelZ`, and `giroX` offsets and replace the existing values on lines 51-53 of `src/main.cpp` :
```
mpu.setYAccelOffset(<REPLACE>);
mpu.setZAccelOffset(<REPLACE>);
mpu.setXGyroOffset(<REPLACE>);
```

### PID
Change the PID constants `Kp`, `Ki`, & `Kd` on lines 21-23 of `src/main.cpp` with whatever values work best. This is mostly a trial & error method.

<br />

## Resources
- https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
- https://github.com/jrowberg/i2cdevlib
- https://docs.arduino.cc/libraries/newping/
- https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
