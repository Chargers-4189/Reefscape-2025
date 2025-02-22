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

  private final RelativeEncoder encoder = actuatorMotor.getEncoder();

  private final DigitalInput upLimitSwitch = new DigitalInput(
    IntakeConstants.kDIO_PORT_TOP
  );

  private final DigitalInput downLimitSwitch = new DigitalInput(
    IntakeConstants.kDIO_PORT_BOTTOM
  );

  public Intake() {}

  /*
  public void actuateUp() {
    if (upLimitSwitch.get() != true) {
      actuatorMotor.set(0.1);
    } else {
      actuatorMotor.set(0);
    }
  }

  public void actuateDown() {
    if (downLimitSwitch.get() != true) {
      actuatorMotor.set(-0.1);
    } else {
      actuatorMotor.set(0);
    }
  }*/

  public boolean getUpLimitSwitch() {
    return upLimitSwitch.get();
  }

  public boolean getDownLimitSwitch() {
    return downLimitSwitch.get();
  }

  public double getEncoder() {
    return -encoder.getPosition();
  }

  public void setPower(double power) {
    power *= IntakeConstants.kPOWER_SCALE;
    /*
    if (downLimitSwitch.get() == true) {
      power = Math.min(power, 0);
    }
    if (downLimitSwitch.get() == false) {
      power = Math.max(power, 0);
    }*/
    actuatorMotor.set(power);
  }

  public void stop() {
    actuatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    actuatorMotor.set(0.010);
  }
}
