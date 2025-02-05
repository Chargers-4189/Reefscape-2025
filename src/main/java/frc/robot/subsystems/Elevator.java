// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private int level = -1;
  private double resistVoltage;

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

  private static final SparkMaxConfig LeftSparkMaxConfig = new SparkMaxConfig();

  /** Creates a new Elevator. */
  public Elevator() {
    zeroEncoder();
    LeftSparkMaxConfig.follow(rightMotor, true);
    leftMotor.configure(
      LeftSparkMaxConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

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

  public double getEncoder() {
    return -encoder.getPosition();
  }
/*
  public double getVelocityMeters() {
    return -encoder.getVelocity() * ElevatorConstants.kROTATIONS_TO_METERS;
  }
*/
  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public boolean getMinLimitSwitch() {
    return minLimitSwitch.get();
  }

  public boolean getMaxLimitSwitch() {
    return maxLimitSwitch.get();
  }

  public void setVoltage(double voltage) {
    // if (getMinLimitSwitch() && voltage < 0) {
    //   voltage = 0;
    // } else if (getMaxLimitSwitch() && voltage > 0) {
    //   voltage = 0;
    // }

    //temporary safety:
    // if (getMinLimitSwitch() || getMaxLimitSwitch()) {
    //   return;
    // }

    // ----- WARNING WARNING -----
    // This is a TEMPORARY SOLUTION. The error with this is that when setPower()
    // method is not running, resistance to gravity will NOT work. Need to add
    // solution in periodic.
    rightMotor.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getEncoder());
  }
}
