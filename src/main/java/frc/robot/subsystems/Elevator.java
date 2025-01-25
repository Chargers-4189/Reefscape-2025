// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class Elevator extends SubsystemBase {
  private int level = -1;
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  private final SparkMax leftMotor = new SparkMax(
    ElevatorConstants.kLEFT_MOTOR_ID,
    MotorType.kBrushless
  );
  private final SparkMax rightMotor = new SparkMax(
    ElevatorConstants.kRIGHT_MOTOR_ID,
    MotorType.kBrushless
  );

  private final DigitalInput minLimitSwitch = new DigitalInput(
    ElevatorConstants.kMIN_DIO_PORT
  );
  private final DigitalInput maxLimitSwitch = new DigitalInput(
    ElevatorConstants.kMAX_DIO_PORT
  );

  private RelativeEncoder encoder = rightMotor.getEncoder();
  private RelativeEncoder encoderL = leftMotor.getEncoder();

  /** Creates a new Elevator. */
  public Elevator() {}

  public int getLevel() {
    return level;
  }

  public void setLevel(int level) {
    this.level = level;
  }

  /*
  public double getEncoderPosition() {
    return encoder.getPosition();
  }*/

  public double getHeightMeters() {
    return encoder.getPosition() * ElevatorConstants.kROTATIONS_TO_METERS;
  }

  public double getVelocityMeters() {
    return encoder.getVelocity() * ElevatorConstants.kROTATIONS_TO_METERS;
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public boolean getMinLimitSwitch() {
    return this.minLimitSwitch.get();
  }

  public boolean getMaxLimitSwitch() {
    return this.maxLimitSwitch.get();
  }

  
  public void setVoltage(double voltage) {
    if (getMinLimitSwitch() && voltage < 0) {
      voltage = 0;
    } else if (getMaxLimitSwitch() && voltage > 0) {
      voltage = 0;
    }

    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public void setPower(double power) {
    if (getMinLimitSwitch() && power < 0) {
      power = 0;
    } else if (getMaxLimitSwitch() && power > 0) {
      power = 0;
    }

    leftMotor.set(power);
    rightMotor.set(-power);
  }

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              voltage -> {
                if (getMinLimitSwitch() && !voltage.gte(Volts.of(0))) {
                  leftMotor.setVoltage(0);
                  rightMotor.setVoltage(0);
                } else if (getMaxLimitSwitch() && voltage.gt(Volts.of(0))) {
                  leftMotor.setVoltage(0);
                  rightMotor.setVoltage(0);
                } else {
                  leftMotor.setVoltage(voltage);
                  rightMotor.setVoltage(voltage);
                }
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the left motors.
                log.motor("elevator-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            leftMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(encoderL.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(encoderL.getVelocity(), MetersPerSecond));
                // Record a frame for the right motors.
                log.motor("elevator-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(encoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(encoder.getVelocity(), MetersPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("Elevator")
              this));
  
  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
