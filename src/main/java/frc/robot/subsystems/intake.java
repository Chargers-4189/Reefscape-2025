// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */

  //Motors
  SparkMax ACTUATOR_MOTOR = new SparkMax(Constants.IntakeConstants.kACTUATOR_MOTOR_ID, MotorType.kBrushless);
  
  //Sensors
  DigitalInput LIMIT_SWITCH_TOP = new DigitalInput(Constants.IntakeConstants.kDIO_PORT_TOP);//magnet up, DIO port
  DigitalInput LIMIT_SWITCH_BOTTOM = new DigitalInput(Constants.IntakeConstants.kDIO_PORT_BOTTOM);//magnet down, DIO port
  SparkAbsoluteEncoder ENCODER; //Encoder plugged into SparkMax

  public Intake() {

    //Gets Encoder from SparkMax
    ENCODER = ACTUATOR_MOTOR.getAbsoluteEncoder();

  }

  //Sets speed / volts of motor
  public void setActuatorSpeed(double speed) {
    ACTUATOR_MOTOR.set(speed);
  }
  
  //encoder
  public double getEncoderPos() {
    return ENCODER.getPosition();
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

  }
}

//Reminder to get Brian/Caleb to look over this so they can tell me how many mistakes I've made.