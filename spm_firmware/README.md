# ALL SPM FIRMWARE

## driver_teensy41
main driver board for motor control loop + inverse kinematics calculations

<<<<<<< Updated upstream
```
driver_teensy41  
└── src/  
=======
driver_teensy41/
└── src/
>>>>>>> Stashed changes
    ├── main.cpp
    ├── teensy41_config.h            # pins config
    ├── kinematics/
    |   ├── machine_params.h       # spm params for kinematics
<<<<<<< Updated upstream
    │   ├── ik.cpp / ik.h          # inverse kinematics
    │   └── fk.cpp / fk.h          # forward kinematics
=======
    │   ├── ik.cpp + .h            # inverse kinematics
    │   └── fk.cpp + .h            # forward kinematics
>>>>>>> Stashed changes
    ├── drivers/
    │   ├── tmc2209.cpp + .h       # stepper control
    │   ├── encoder.cpp + .h       # quadrature decoder
    │   └── stepper.cpp + .h       # fastaccelsteper wrapper
    ├── control/
    │   └── pid.cpp + .h           # motor pid loop
    └── comms/
<<<<<<< Updated upstream
        └── ros_interface.cpp / .h # micro-ROS pub/sub
```
=======
        └── ros_interface.cpp + .h # micro-ROS pub/sub

>>>>>>> Stashed changes

## platform_esp32c6
board used on end effector platform to wirelessly transmit data from bno085 imu via esp32now

## receiver_esp32wroom
board used as dongle to receive imu data from platform via esp32now
