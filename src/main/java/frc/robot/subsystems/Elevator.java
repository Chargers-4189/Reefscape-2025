// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

   SparkMax leftActuatorSparkMax = new SparkMax(
    ElevatorConstants.kLEFT_MOTOR_ID, // ID CHANGE HERE
    MotorType.kBrushless
    ); 
 
     SparkMax rightActuatorSparkMax = new SparkMax(
    ElevatorConstants.kRIGHT_MOTOR_ID, // ID CHANGE HERE
    MotorType.kBrushless
    ); 

    SparkAbsoluteEncoder leftActuatorEncoder =  leftActuatorSparkMax.getAbsoluteEncoder();
    SparkAbsoluteEncoder rightActuatorEncoder =  rightActuatorSparkMax.getAbsoluteEncoder();

    DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.kMAX_LIMIT_DIO);
    DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.kMIN_LIMIT_DIO);
    
  /** Creates a new Elevator. */
  public Elevator() {}

  public void moveElevator(double speed){
      //leftActuatorSparkMax.set(speed);
      rightActuatorSparkMax.set(-speed);
  }
  
  public void stayStill(){
      leftActuatorSparkMax.set(ElevatorConstants.kGRAVITY_VOLTS);
      rightActuatorSparkMax.set(ElevatorConstants.kGRAVITY_VOLTS);
  }

  public double getEncoderValue(){
      return(rightActuatorEncoder.getPosition());
  }

  public boolean getTopLimitSwitch(){
      return(topLimitSwitch.get());
  }

  public boolean getBottomLimitSwitch(){
      return(bottomLimitSwitch.get());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}