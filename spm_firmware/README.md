# ALL SPM FIRMWARE

## driver_teensy41
main driver board for motor control loop + inverse kinematics calculations

```
driver_teensy41/
└── src/
    ├── main.cpp
    ├── teensy41_config.h           # pins config
    ├── machine_params.h            # spm params for kinematics
    ├── kinematics/
    │   ├── ik.cpp + .h             # inverse kinematics
    │   └── fk.cpp + .h             # forward kinematics
    ├── drivers/
    │   ├── tmc2209.cpp + .h        # stepper control
    │   ├── encoder.cpp + .h        # quadrature decoder
    │   └── stepper.cpp + .h        # teensystep4 wrapper
    ├── control/
    │   └── pid.cpp + .h            # motor pid loop
    └── comms/
        └── ros_interface.cpp + .h  # micro-ROS pub/sub
```

## platform_esp32c6
board used on end effector platform to wirelessly transmit data from bno085 imu via esp32now

## receiver_esp32wroom
board used as dongle to receive imu data from platform via esp32now
