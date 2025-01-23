// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.CoralEffectorConstants;

public class CoralEffector extends SubsystemBase {

  private final SparkMax leftMotor = new SparkMax(
    CoralEffectorConstants.kLEFT_MOTOR_ID,
    MotorType.kBrushless
  );

  private final SparkMax rightMotor = new SparkMax(
    CoralEffectorConstants.kRIGHT_MOTOR_ID,
    MotorType.kBrushless
  );

  private final LaserCan intakeSensor = new LaserCan(CoralEffectorConstants.kINTAKE_SENSOR_ID);
  private final LaserCan outtakeSensor = new LaserCan(CoralEffectorConstants.kOUTTAKE_SENSOR_ID);

  /** Creates a new CoralEffector. */
  public CoralEffector() {
    try {
      intakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      intakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      intakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
      outtakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      outtakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      outtakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void set(double leftMotorPower, double rightMotorPower) {
    leftMotor.set(leftMotorPower);
    rightMotor.set(rightMotorPower);
  }

  public boolean getIntakeSensor() {
    LaserCan.Measurement inMeasurement = intakeSensor.getMeasurement();
    if (inMeasurement != null && inMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (inMeasurement.distance_mm < CoralEffectorConstants.kMEASURE_THRESHOLD);
    } else {
      System.out.println("ERROR! Coral Effector Intake LaserCan measurement is invalid");
    }
    return false;
  }

  public boolean getOuttakeSensor() {
    LaserCan.Measurement outMeasurement = outtakeSensor.getMeasurement();
    if (outMeasurement != null && outMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (outMeasurement.distance_mm < CoralEffectorConstants.kMEASURE_THRESHOLD);
    } else {
      System.out.println("ERROR! Coral Effector Outtake LaserCan measurement is invalid");
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
