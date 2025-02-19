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

  private final DigitalInput topLimitSwitch = new DigitalInput(
    IntakeConstants.kDIO_PORT_TOP
  );

  private final DigitalInput bottomLimitSwitch = new DigitalInput(
    IntakeConstants.kDIO_PORT_BOTTOM
  );
  public Intake() {}
    public void ActuateForward(){
      if(topLimitSwitch.get() != true){
      actuatorMotor.set(0.1);
      }
      else{
        actuatorMotor.set(0);
      }
    }
    public void ActuateBackward(){
      if(bottomLimitSwitch.get() != true){
        actuatorMotor.set(-0.1);
      }
      else{
        actuatorMotor.set(0);
      }
    }
  
    public void StopActuating(){
      actuatorMotor.set(0);
    }
    public boolean getTopLimitSwitch() {
      return topLimitSwitch.get();
    }
    public boolean getBottomLimitSwitch() {
      return bottomLimitSwitch.get();
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
