// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.util.Elastic.ElasticClimber;



public class Climber extends SubsystemBase {
  private final SparkMax climberMotor = new SparkMax(
    ClimberConstants.kMOTOR_ID,
    MotorType.kBrushless
  );
  
  /** Creates a new Climber. */
  public Climber() {
    
  }

  public void setPower(double power){
    power = Math.min(power, 1);
    power = Math.max(power, -1);
    power *= ElasticClimber.kMAX_POWER.get();

    climberMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}