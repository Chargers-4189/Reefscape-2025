// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
public final class Constants {

  public static final double ROBOT_MASS = Units.lbsToKilograms(119); //(148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(
    new Translation3d(0, 0, Units.inchesToMeters(8)),
    ROBOT_MASS
  );
  public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static class VisionConstants {

    public static final Transform3d flCamPose = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(15.25),
        Units.inchesToMeters(6.25),
        Units.inchesToMeters(9)
      ),
      new Rotation3d(0, Math.toRadians(0), 0)
    );

    public static final Transform3d frCamPose = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(15.25),
        Units.inchesToMeters(-6.25),
        Units.inchesToMeters(9)
      ),
      new Rotation3d(0, Math.toRadians(0), 0)
    );

    public static final Transform3d bkCamPose = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0),
        Units.inchesToMeters(0)
      ),
      new Rotation3d(0, Math.toRadians(0), Math.toRadians(180.0))
    );

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025ReefscapeWelded
    );
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int secondaryController = 1;
    public static final double kSecondaryDeadband = 0.1;
  }

  public static class CoralEffectorConstants {

    //Subsystem
    public static final int kMAIN_MOTOR_ID = 31;
    public static final int kINTAKE_SENSOR_DIO = 6;
    public static final int kOUTTAKE_SENSOR_ANALOG = 0;
    //Intake
    public static final int kNOISE_COUNT_LIMIT = 3;
    public static final int kMILISECONDS_EXTRA_PULLBACK = 0;
    //Algae
    public static final double kALGAE_POWER = .6;
    public static final int kEJECT_ALGAE_MILISECONDS = 1500;
    //Outtake
    public static final double kCORAL_POWER = .1;
    public static final int kMILISECONDS_OUTTAKE = 2;

    public static final double kSECONDARY_OUT_POWER = .05;
    public static final double kSECONDARY_IN_POWER = .05;
  }

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

    public static final double[] kHEIGHTS = {
      .01,
      .01,
      7.9,
      26.1,
      53.8,
      3,
      19.5,
    }; // Intake, L1, L2, L3, L4, Algae Low, Algae High

    public static final double kGRAVITY_VOLTS = .4;
    public static final double kPROPORTIONAL_VOLTS = .8;
    public static final double kMAX_VOLTS = 12;
    public static final double kMAX_VOLT_CHANGE_PER_SECOND = 40;

    public static final double kTOLERANCE = 1.4;
    public static final int kSLIGHTLY_DOWN_TIMEOUT = 500;
    public static final int kTIMEOUT = 1000;
  }

  public static final class IntakeConstants {

    public static final int kACTUATOR_MOTOR_ID = 41;
    public static final int kDIO_PORT_TOP = 2;
    public static final int kDIO_PORT_BOTTOM = 3;
    public static final double kPOWER_SCALE = .1;

    //public static final double kGRAVITY_VOLTS = .01;

    //public static final double kUP_ENCODER = 22.5;
    public static final double kPOWER = .6;
  }

  /*
  public static final class AutoConstants {
    
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );
  }*/

  public static class AlignmentConstants {

    public static final double kDIST_FROM_REEF = .19;
    public static final double kDIST_OFFSET = Units.inchesToMeters(11.338);

    public static final double kROTATION_TIMEOUT = .5; //Seconds
    public static final double kPOSITION_TIMEOUT = 2.5; //Seconds
    
    public static final double kROTATION_TOLERANCE = 4; //Degrees

    public static final double kMAX_SPEED_X = .6;
    public static final double kMAX_SPEED_Y = .6;
    public static final double kMAX_SPEED_ANGLE = 1;
    public static final double kOVERARCHING_TIMEOUT = 3; //seconds

    public static final double kP_X = 0;
    public static final double kP_Y = 0;
    public static final double kP_ANGLE = 0;

    public static final double kI_X = 0;
    public static final double kI_Y = 0;
    public static final double kI_ANGLE = 0;

    public static final double kD_X = 0;
    public static final double kD_Y = 0;
    public static final double kD_ANGLE = 0;

    /* 
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared =
      Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond,
      kMaxAngularSpeedRadiansPerSecondSquared
    );

    public static final PathConstraints kPATH_CONSTRAINTS = new PathConstraints(
      3.0, 4.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720)
    );
    */
  }

  public static class HumanDriveConstants {

    public static final double kDRIVE_POWER = .4; //Decreased for safety: prevent tipping
    public static final double kROTATIONAL_POWER = .8;
    public static final double kDRIVE_EXPONENT = 3;
    public static final double kROTATIONAL_EXPONENT = 1;
  }

  public static class ClimberConstants {

    public static final int kMIN_LIMIT_DIO = 4;
    public static final int kMAX_LIMIT_DIO = 5;

    public static final int kMOTOR_ID = 51;

    public static final double kMAX_POWER = 0.5;
    public static final double kMAX_UP_POWER = .05;
    public static final double kMAX_DOWN_POWER = .05;
  }
}
