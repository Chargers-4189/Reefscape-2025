// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = Units.lbsToKilograms(119);//(148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class CoralActuatorConstants
  {
    public static class ElevatorConstants {

      public static final int kLEFT_MOTOR_ID = 11;
      public static final int kRIGHT_MOTOR_ID = 12;
  
      public static final int kMIN_LIMIT_DIO = 0;
      public static final int kMAX_LIMIT_DIO = 1;
  
      public static final int kMaxCurrentDriveMotor = 50;
  
      // public static final double kROTATIONS_TO_METERS = 1.757 * Math.PI * 2.54 /
      // 100;
  
      // public static final double[] HEIGHTS_METERS = { .720, .700, .776, 1.179,
      // 1.829}; //Intake, L1, L2, L3, L4
  
      public static final double[] kHEIGHTS = { .01, .01, 7.9, 26.1, 53.8, 3, 19.5}; // Intake, L1, L2, L3, L4, Algae Low, Algae High
  
      public static final double kGRAVITY_VOLTS = .4;
      public static final double kPROPORTIONAL_VOLTS = .8;
      public static final double kMAX_VOLTS = 12;
      public static final double kMAX_VOLT_CHANGE_PER_SECOND = 40;
  
      public static final double kTOLERANCE = 1.4;
      public static final double kSLIGHTLY_DOWN_TIMEOUT = .5;
      public static final int kTIMEOUT = 1;
    }
    public static class CoralEffectorConstants {
      
    }
  }
}
