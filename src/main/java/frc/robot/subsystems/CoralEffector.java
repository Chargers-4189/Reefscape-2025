// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants.CoralEffectorConstants;

public class CoralEffector extends SubsystemBase {

  private final SparkMax mainMotor = new SparkMax(
    CoralEffectorConstants.kLEFT_MOTOR_ID,
    MotorType.kBrushless
  );

  private AbsoluteEncoder encoder = mainMotor.getAbsoluteEncoder();

  private final LaserCan intakeSensor = new LaserCan(CoralEffectorConstants.kINTAKE_SENSOR_ID);
  //private final LaserCan outtakeSensor = new LaserCan(CoralEffectorConstants.kOUTTAKE_SENSOR_ID);

  public String state = "empty";

  /** Creates a new CoralEffector. */
  public CoralEffector() {
    try {
      intakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      intakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 2, 2));
      intakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void setPower(double mainMotorPower) {
    mainMotor.set(-mainMotorPower);
  }

  public void stop() {
    mainMotor.set(0);
  }
  

  public boolean getIntakeSensor() {
    LaserCan.Measurement inMeasurement = intakeSensor.getMeasurement();
    //System.out.println(inMeasurement.distance_mm);
    /*
    if (inMeasurement == null) {
      System.out.print("null ");
    }
    if (inMeasurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.print("invalid ");
    }
    System.out.println(inMeasurement.distance_mm);
    */
    return (inMeasurement.distance_mm < CoralEffectorConstants.kMEASURE_THRESHOLD);
  }
  /*
  public boolean getOuttakeSensor() {
    LaserCan.Measurement outMeasurement = outtakeSensor.getMeasurement();
    if (outMeasurement != null && outMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return (outMeasurement.distance_mm < CoralEffectorConstants.kMEASURE_THRESHOLD);
    } else {
      System.out.println("ERROR! Coral Effector Outtake LaserCan measurement is invalid");
    }
    return false;
  }*/

  public double getAbsoluteEncoderValue(){
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    System.out.print(state);
    System.out.print(" ");i
    System.out.println(getIntakeSensor());
    */
  }
}
