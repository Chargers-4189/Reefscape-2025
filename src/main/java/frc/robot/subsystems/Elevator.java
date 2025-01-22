// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  public enum Level {
    INTAKE,
    L1,
    L2,
    L3,
    L4,
  }

  private Level level = Level.INTAKE;

  private final SparkMax leftMotor = new SparkMax(
    Constants.Elevator.kLEFT_MOTOR_ID,
    MotorType.kBrushless
  );
  private final SparkMax rightMotor = new SparkMax(
    Constants.Elevator.kRIGHT_MOTOR_ID,
    MotorType.kBrushless
  );

  private final DigitalInput minLimitSwitch = new DigitalInput(
    Constants.Elevator.kMIN_DIO_PORT
  );
  private final DigitalInput maxLimitSwitch = new DigitalInput(
    Constants.Elevator.kMAX_DIO_PORT
  );

  private RelativeEncoder encoder = rightMotor.getEncoder();

  /** Creates a new Elevator. */
  public Elevator() {}

  public Level getLevel() {
    return level;
  }

  public void setLevel(Level level) {
    this.level = level;
  }

  public double getEncoder() {
    return encoder.getPosition();
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

  public void set(double power) {
    if (getMinLimitSwitch() && power < 0) {
      power = 0;
    } else if (getMaxLimitSwitch() && power > 0) {
      power = 0;
    }

    leftMotor.set(power);
    rightMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
