// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralEffectorConstants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralEffector extends SubsystemBase {
  /** Creates a new CoralEffector. */
 SparkMax ActuatorSparkMax = new SparkMax(
    CoralEffectorConstants.kMAIN_MOTOR_ID, // ID CHANGE HERE
    MotorType.kBrushless
  ); 

  SparkAbsoluteEncoder ActuatorEncoder =  ActuatorSparkMax.getAbsoluteEncoder();

  DigitalInput digitalSensor = new DigitalInput(CoralEffectorConstants.kINTAKE_SENSOR_DIO);
  AnalogInput analogSensor = new AnalogInput(CoralEffectorConstants.kOUTTAKE_SENSOR_ANALOG);



  public CoralEffector() {}
   
  public void moveEffector(){
    ActuatorSparkMax.set(-0.2);
  }

  public void moveEffectorBackwards(){
    ActuatorSparkMax.set(0.05);
  }

  public void stopEffector(){
    ActuatorSparkMax.set(0);
  }

  public boolean getDigitalSensor(){
    return(digitalSensor.get());
  }

  public boolean getAnalogSensor(){
    return((analogSensor.getValue() < 3500));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
