# Swerve Drive YAGSL Configuration

### Details
| Configuration                                              | Value                | Relevance                                                                            |
|------------------------------------------------------------|----------------------|--------------------------------------------------------------------------------------|
| Drive Gear Ratio                                           | 5.50:1               | 12 Teeth pinion; Low gear ratio                                                      |
| Steering Gear Ratio                                        | ~46.42:1 or 9424:203 | MAXSwerve with NEO550                                                                |
| Absolute Encoder Ticks Per Revolution                      | 1                    | Unknown                                                                              |
| CAN Bus Name                                               | rio                  | Connected directly to ROBORIO via CAN                                                |
| CAN IDs                                                    | DOCUMENTED           | Firmware configuration                                                               |
| Gyro Connection                                            | navx_spi             | Connected directly to ROBORIO via MXP                                                |
| Drive Motor Inversion                                      | false                | Based on testing > Inversion must turn wheels forward                                |
| Steering Motor Inversion                                   | true                 | Based on testing > Inversion must turn wheels clockwise                              |
| Absolute Encoder Inversion                                 | false                | Based on testing > Increase value along with steering motor movements.               |
| Gyro Inversion                                             | false                | Based on testing > The gyroscope needs to be counter-clockwise positive.             |
| Absolute Encoder Offset                                    | 0                    | Used to straighten out all modules; MAXSwerve needs configuration on firmware itself |
| Motor Controller PID                                       | default              | Configured via example code; further tuning may be needed                            |
| Distance from center of robot to center of wheel in inches | DOCUMENTED           | Used for SwerveDriveKinematics                                                       |

Steering gear ratio full value: `46.423645320197`

drive conversion factor: `0.00000531315`

#### Sources
- https://docs.wpilib.org/en/stable/docs/hardware/sensors/serial-buses.html
- https://www.revrobotics.com/rev-21-3005/

#### Physical Measurements
- 14.508 Inches X
- 12.018 Inches Y

CAN ID* designation:
| Left        | Forward | Right       |
|-------------|---------|-------------|
| D: 24 T: 25 |         | D: 22 T: 23 |
| D: 26 T: 27 |         | D: 20 T: 21 |

> If your absolute encoder is attached to your SparkMAX, use the function SwerveDrive.pushOffsetsToEncoders() for the best performance. This sets the onboard PID sensor to the attached encoder!