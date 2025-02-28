// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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

  private final DigitalInput intakeSensor = new DigitalInput(0);
  //private final LaserCan outtakeSensor = new LaserCan(CoralEffectorConstants.kOUTTAKE_SENSOR_ID);
  private final DigitalInput outtakeSensor = new DigitalInput(0);


  public String state = "empty";

  /** Creates a new CoralEffector. */
  public CoralEffector() {
  }

  public void setPower(double mainMotorPower) {
    mainMotor.set(-mainMotorPower);
  }

  public void stop() {
    mainMotor.set(0);
  }
  

  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }
  
  public boolean getOuttakeSensor() {
    return outtakeSensor.get();
  }

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
