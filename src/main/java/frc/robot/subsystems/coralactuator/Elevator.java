// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralactuator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CoralActuatorConstants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private int level = 0;

  private final SparkMax leftMotor = new SparkMax(
    ElevatorConstants.kLEFT_MOTOR_ID,
    MotorType.kBrushless
  );
  private final SparkMax rightMotor = new SparkMax(
    ElevatorConstants.kRIGHT_MOTOR_ID,
    MotorType.kBrushless
  );

  private final DigitalInput minLimitSwitch = new DigitalInput(
    ElevatorConstants.kMIN_LIMIT_DIO
  );
  private final DigitalInput maxLimitSwitch = new DigitalInput(
    ElevatorConstants.kMAX_LIMIT_DIO
  );

  private RelativeEncoder encoder = leftMotor.getEncoder();

  private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
  private static final SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();
  //private static final ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0, 0, 0, 0);
  //private static final ElevatorFeedforward test = new ElevatorFeedforward(0, 0, 0);
  //private static final ElevatorFeedforward test1 = new ElevatorFeedforward(0, 0, 0, 0);
  /** Creates a new Elevator. */
  //private DCMotor gearBox = DCMotor.getNEO(2).withReduction(12);
  //private LinearSystem<N2,N1,N2> Elevator = LinearSystemId.createElevatorSystem(gearBox, level, level, level);
  //private SparkMaxSim sparkMaxSim;
  //private ElevatorSim elevatorSim;
  public Elevator() {
    zeroEncoder();
    leftSparkMaxConfig.follow(rightMotor, true);
    leftSparkMaxConfig.idleMode(IdleMode.kBrake);
    rightSparkMaxConfig.idleMode(IdleMode.kBrake);
    leftMotor.configure(
      leftSparkMaxConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    rightMotor.configure(
      rightSparkMaxConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
    if(Robot.isSimulation()){
      //this.sparkMaxSim = new SparkMaxSim(leftMotor, this.gearBox);
      //elevatorSim = new ElevatorSim(Elevator, gearBox, level, level, getMaxLimitSwitch(), level, null);
    }
    leftMotor.set(0);
  }

  public int getLevel() {
    return level;
  }

  public void setLevel(int level) {
    this.level = level;
  }

  public double getEncoder() {
    return -encoder.getPosition();
  }

  /*
  public double getVelocityMeters() {
    return -encoder.getVelocity() * ElevatorConstants.kROTATIONS_TO_METERS;
  }
*/
  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public boolean getMinLimitSwitch() {
    return !minLimitSwitch.get();
  }

  public boolean getMaxLimitSwitch() {
    return !maxLimitSwitch.get();
  }

  public void setVoltage(double voltage) {
    if (getMinLimitSwitch()) {
      if (voltage < -.2) {
        voltage = -.2;
      }
    }
    if (getMaxLimitSwitch()) {
      if (voltage > .2) {
        voltage = .2;
      }
    }
    rightMotor.setVoltage(-voltage - ElevatorConstants.kGRAVITY_VOLTS);
  }

  public void setVoltageNoGravity(double voltage) {
    if (getMinLimitSwitch()) {
      if (voltage < -.3) {
        voltage = -.3;
      }
    }
    if (getMaxLimitSwitch()) {
      if (voltage > .3) {
        voltage = .3;
      }
    }
    rightMotor.setVoltage(-voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getMinLimitSwitch()) {
      zeroEncoder();
    }
    /*
    System.out.print("Bottom: " + getMinLimitSwitch());
    System.out.print("   Top: " + getMaxLimitSwitch());
    System.out.print("   Encoder: " + getEncoder());
    System.out.println();
    */

  }
}
