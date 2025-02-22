// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

//ADD CONSTANTS
public class Intake extends SubsystemBase {
  private DoubleEntry kGRAVITY_VOLTS;
  private DoubleEntry kMAX_POWER;

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

  public Intake() {
    zeroEncoder();
    NetworkTableInstance networkInstance = NetworkTableInstance.getDefault();
    NetworkTable datatable = networkInstance.getTable("intakeConstants");
    kGRAVITY_VOLTS =
      datatable
        .getDoubleTopic("GRAVITY_VOLTS")
        .getEntry(IntakeConstants.kGRAVITY_VOLTS);
    
        kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
    kMAX_POWER =
      datatable
        .getDoubleTopic("MAX_POWER")
        .getEntry(IntakeConstants.kMAX_POWER);
  
      kGRAVITY_VOLTS.set(kGRAVITY_VOLTS.get());
      kMAX_POWER.set(kMAX_POWER.get());
    
  }

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
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public void setPower() {
    /*
    if (downLimitSwitch.get() == true) {
      power = Math.min(power, 0);
    }
    if (upLimitSwitch.get() == true) {
      power = Math.max(power, 0);
    }*/
    //power = Math.min(power, kMAX_POWER.get());
    //power = Math.max(power, - kMAX_POWER.get());
    actuatorMotor.set(kGRAVITY_VOLTS.get());
  }

  public void stop() {
    actuatorMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getEncoder());
  }
}
