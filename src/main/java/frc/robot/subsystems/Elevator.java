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

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  public double kPROPORTIONAL_VOLTS = ElevatorConstants.kPROPORTIONAL_VOLTS;
  public double kMAX_VOLT_CHANGE_PER_SECOND = ElevatorConstants.kMAX_VOLT_CHANGE_PER_SECOND;
  public double kMAX_VOLTS = ElevatorConstants.kMAX_VOLTS;
  public double kGRAVITY_VOLTS = ElevatorConstants.kGRAVITY_VOLTS;

  private int level = -1;

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

    SmartDashboard.putData("Elevator Control Vars", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ElevatorControl");

        builder.addDoubleProperty("Proportional Voltage", null, (x) -> kPROPORTIONAL_VOLTS = x);
        builder.addDoubleProperty("Max Voltage", null, (x) -> kMAX_VOLTS = x);
        builder.addDoubleProperty("Max Voltage Change Per Second", null, (x) -> kMAX_VOLT_CHANGE_PER_SECOND = x);
        builder.addDoubleProperty("Gravity Voltage", null, (x) -> kMAX_VOLT_CHANGE_PER_SECOND = x);
      }
    });
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
