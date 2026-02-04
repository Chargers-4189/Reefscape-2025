// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */

  //Motors
  SparkMax ACTUATOR_MOTOR = new SparkMax(Constants.IntakeConstants.kACTUATOR_MOTOR_ID, MotorType.kBrushless);
  
  //Sensors
  DigitalInput LIMIT_SWITCH_TOP = new DigitalInput(Constants.IntakeConstants.kDIO_PORT_TOP);//magnet up, DIO port
  DigitalInput LIMIT_SWITCH_BOTTOM = new DigitalInput(Constants.IntakeConstants.kDIO_PORT_BOTTOM);//magnet down, DIO port
  RelativeEncoder ENCODER = ACTUATOR_MOTOR.getEncoder();//Gets Encoder from SparkMax


  public Intake() {
    
  }

  //Sets speed of motor
  public void setActuatorSpeed(double speed) {
    ACTUATOR_MOTOR.set(speed);
  }
  
  //encoder
  public double getEncoderPos() {
    return ENCODER.getPosition();
  }

  public void resetEncoder() {
    ENCODER.setPosition(0.00);
  }

  //switches
  public boolean getTopSwitch() {
    return LIMIT_SWITCH_TOP.get();
  }

  public boolean getBottomSwitch() {
    return LIMIT_SWITCH_BOTTOM.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    System.out.println(ENCODER.getPosition());
    //System.out.println(LIMIT_SWITCH_TOP.get());
    //System.out.println(LIMIT_SWITCH_BOTTOM.get());
  }
}