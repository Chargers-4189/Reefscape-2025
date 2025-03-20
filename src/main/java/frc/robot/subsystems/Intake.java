// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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

  private RelativeEncoder encoder = actuatorMotor.getEncoder();

  public Intake() {}
    public void setPower(double power) {
      if (getBottomLimitSwitch()) {
        if (power < 0) {
          power = 0;
        }
      }
      if (getTopLimitSwitch()) {
        if (power > .05) {
          power = .05;
        }
      }
      actuatorMotor.set(power);
    }
  
    public void stop(){
      actuatorMotor.set(0);
    }

    public boolean getTopLimitSwitch() {
      return !topLimitSwitch.get();
    }
    public boolean getBottomLimitSwitch() {
      return !bottomLimitSwitch.get();
    }

    public double getEncoder() {
      return encoder.getPosition();
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    System.out.print("Bottom: " + getBottomLimitSwitch());
    System.out.print("   Top: " + getTopLimitSwitch());
    System.out.println();
    */
  }
}