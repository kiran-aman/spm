# ALL SPM FIRMWARE

## driver_teensy41
main driver board for motor control loop + inverse kinematics calculations

```
driver_teensy41/
├── include/
│   ├── machine_params.h            # spm physical parameters
│   └── teensy41_config.h           # teensy 4.1 pins
├── src/
│   ├── main.cpp
│   ├── kinematics/
│   │   ├── ik.cpp + .h             # inverse kinematics
│   │   └── trajectory.cpp + .h     # pins config
│   ├── lib/
│   │   ├── tmc2209.cpp + .h        # stepper control
│   │   ├── encoder.cpp + .h        # quadrature decoder
│   │   └── stepper.cpp + .h        # teensystep4 wrapper
│   ├── control/
│   │   └── pid.cpp + .h            # motor pid loop
│   └── comms/
│       └── ros_interface.cpp + .h  # micro-ROS pub/sub
└── lib/
    ├── tmc2209/                    # stepper control
    ├── encoder/                    # quadrature decoder
    └── stepper_driver/             # teensystep4 wrapper

```

## platform_esp32c6
board used on end effector platform to wirelessly transmit data from bno085 imu via esp32now

## receiver_esp32wroom
board used as dongle to receive imu data from platform via esp32now
