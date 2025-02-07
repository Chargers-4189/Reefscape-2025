// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public DoubleEntry kGRAVITY_VOLTS;
  public DoubleEntry kPROPORTIONAL_VOLTS;
  public DoubleEntry kMAX_VOLTS;
  public DoubleEntry kMAX_VOLT_CHANGE_PER_SECOND;
  public DoubleEntry kTOLERANCE;
  public DoubleArrayEntry kHEIGHTS;

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

    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    NetworkTable datatable = networkInstance.getTable("elevatorConstants");
    kGRAVITY_VOLTS = datatable.getDoubleTopic("GRAVITY_VOLTS").getEntry(ElevatorConstants.kGRAVITY_VOLTS);
    kMAX_VOLTS = datatable.getDoubleTopic("MAX_VOLTS").getEntry(ElevatorConstants.kMAX_VOLTS);
    kMAX_VOLT_CHANGE_PER_SECOND = datatable.getDoubleTopic("MAX_VOLT_CHANGE_PER_SECOND").getEntry(ElevatorConstants.kMAX_VOLT_CHANGE_PER_SECOND);
    kPROPORTIONAL_VOLTS = datatable.getDoubleTopic("PROPORTIONAL_VOLTS").getEntry(ElevatorConstants.kPROPORTIONAL_VOLTS);
    kTOLERANCE = datatable.getDoubleTopic("TOLERANCE").getEntry(ElevatorConstants.kTOLERANCE);
    kHEIGHTS = datatable.getDoubleArrayTopic("HEIGHTS").getEntry(ElevatorConstants.kHEIGHTS);
    
    kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
    kPROPORTIONAL_VOLTS.set(kPROPORTIONAL_VOLTS.get());
    kMAX_VOLTS.set(kMAX_VOLTS.get());
    kMAX_VOLT_CHANGE_PER_SECOND.set(kMAX_VOLT_CHANGE_PER_SECOND.get());
    kTOLERANCE.set(kTOLERANCE.get());
    kHEIGHTS.set(kHEIGHTS.get());
  }

  public int getLevel() {
    return level;
  }

  public void setLevel(int level) {
    this.level = level;
  }

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

    rightMotor.setVoltage(-voltage - kGRAVITY_VOLTS.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getEncoder());

    System.out.println(getEncoder());
  }
}
