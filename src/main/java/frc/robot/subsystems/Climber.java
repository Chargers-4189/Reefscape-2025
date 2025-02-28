// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.util.Elastic.ElasticClimber;



public class Climber extends SubsystemBase {
  private final SparkMax climberMotor = new SparkMax(
    ClimberConstants.kMOTOR_ID,
    MotorType.kBrushless
  );
  private final DigitalInput minLimitSwitch = new DigitalInput(ClimberConstants.kMIN_LIMIT_DIO);

  private final DigitalInput maxLimitSwitch = new DigitalInput(ClimberConstants.kMAX_LIMIT_DIO);

  /** Creates a new Climber. */
  public Climber() {
    
  }

  public void setPower(double power){
    power = Math.min(power, 1);
    power = Math.max(power, -1);
    power *= ElasticClimber.kMAX_POWER.get();

    if (getMaxLimit()) {
      power = Math.min(0, power);
    }
    if (getMinLimit()) {
      power = Math.max(0, power);
    }

    climberMotor.set(power);
  }

  public boolean getMinLimit() {
    return minLimitSwitch.get();
  }
  public boolean getMaxLimit() {
    return maxLimitSwitch.get();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Climber min limit: " + getMinLimit() + "   max limit: " + getMaxLimit());
  }


}