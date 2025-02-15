// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
//ADD CONSTANTS
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  private final SparkMax actuatorMotor = new SparkMax(
    IntakeConstants.kACTUATOR_MOTOR_ID,
    MotorType.kBrushless
  );

  private final DigitalInput limitSwitch = new DigitalInput(
    IntakeConstants.kDIO_PORT
  );
  public Intake() {}
    public void ActuateForward(){
      actuatorMotor.set(0.1);
    }
    public void ActuateBackward(){
      actuatorMotor.set(-0.1);
    }
  
    public void StopActuating(){
      actuatorMotor.set(0);
    }
    public boolean getLimitSwitch() {
      return limitSwitch.get();
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
