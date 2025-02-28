// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.CoralEffectorConstants;

public class CoralEffector extends SubsystemBase {

  private final SparkMax mainMotor = new SparkMax(
    CoralEffectorConstants.kMAIN_MOTOR_ID,
    MotorType.kBrushless
  );

  private AbsoluteEncoder encoder = mainMotor.getAbsoluteEncoder();

  private final DigitalInput intakeSensor = new DigitalInput(6);
  //private final LaserCan outtakeSensor = new LaserCan(CoralEffectorConstants.kOUTTAKE_SENSOR_ID);
  //private final DigitalInput outtakeSensor = new DigitalInput(6);
  private final AnalogInput outtakeSensor = new AnalogInput(0);


  public String state = "empty";

  private int intakeOnCounter = 0;
  private int intakeOffCounter = 0;
  private int outtakeOnCounter = 0;
  private int outtakeOffCounter = 0;
  private boolean intakeOn;
  private boolean outtakeOn;

  /** Creates a new CoralEffector. */
  public CoralEffector() {
  }

  public void setPower(double mainMotorPower) {
    mainMotor.set(-mainMotorPower);
  }

  public void stop() {
    mainMotor.set(0);
  }
  

  public boolean getIntakeSensor() {
    return intakeOn;
  }
  
  public boolean getOuttakeSensor() {
    return outtakeOn;
  }

  public double getAbsoluteEncoderValue(){
    return encoder.getPosition();
  }

  private void updateSensors() {
    //Update intake sensor counters
    if (intakeSensor.get()) {
      intakeOnCounter++;
      intakeOffCounter = 0;
    } else {
      intakeOffCounter++;
      intakeOnCounter = 0;
    }

    //update outtake sensor counters
    if (outtakeSensor.getVoltage() > 2) {
      outtakeOnCounter++;
      outtakeOffCounter = 0;
    } else {
      outtakeOffCounter++;
      outtakeOnCounter = 0;
    }

    //update intake boolean
    if (intakeOnCounter >= 3) {
      intakeOn = true;
    } else if (intakeOffCounter >= 3) {
      intakeOn = false;
    }
    
    //update outtake boolean
    if (outtakeOnCounter >= 3) {
      outtakeOn = true;
    } else if (outtakeOffCounter >= 3) {
      outtakeOn = false;
    }
  }

  @Override
  public void periodic() {
    updateSensors();
    //System.out.print("intakeSensor:" + getIntakeSensor());
    //System.out.print("   outtakeSensor:" + getOuttakeSensor());
    //System.out.print("   intakeCounters(On,Off):" + intakeOnCounter + "," + intakeOffCounter);
    //System.out.print("   outtakeCounters(On,Off):" + outtakeOnCounter + "," + outtakeOffCounter);
    //System.out.println();

  }
}
