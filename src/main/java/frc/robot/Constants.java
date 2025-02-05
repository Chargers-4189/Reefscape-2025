// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static class CoralEffectorConstants {

    public static final double kPLACE_CORAL_ANGLE = -1; //Find the optimal angle for placing coral in absolute encoder
    public static final double kDEFAULT_CORAL_EFFECTER_ANGLE = -1; //Find the optimal default angle in absolute encoder
    public static final int kACTUATOR_ID = 33;
    public static final int kLEFT_MOTOR_ID = 31;
    public static final int kRIGHT_MOTOR_ID = 32;
    public static final int kINTAKE_SENSOR_ID = 34;
    public static final int kOUTTAKE_SENSOR_ID = 35;
    public static final int kMEASURE_THRESHOLD = 20; //milimeters
  }

  public static final class DriveConstants {

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    // ----- TODO: GET CHASSIS MEASUREMENT -----
    public static final double kTrackWidth = Units.inchesToMeters(29.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 24;
    public static final int kRearLeftDrivingCanId = 26;
    public static final int kFrontRightDrivingCanId = 22;
    public static final int kRearRightDrivingCanId = 20;

    public static final int kFrontLeftTurningCanId = 25;
    public static final int kRearLeftTurningCanId = 27;
    public static final int kFrontRightTurningCanId = 23;
    public static final int kRearRightTurningCanId = 21;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps =
      NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters =
      kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
      (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) /
      kDrivingMotorReduction;
  }

  public static class ElevatorConstants {

    public static final int kLEFT_MOTOR_ID = 11;
    public static final int kRIGHT_MOTOR_ID = 12;
    public static final int kMIN_DIO_PORT = 0;
    public static final int kMAX_DIO_PORT = 1;
    public static final int kMaxCurrentDriveMotor = 50;

    //public static final double kROTATIONS_TO_METERS = 1.757 * Math.PI * 2.54 / 100;

    //public static final double[] HEIGHTS_METERS = { .720, .700, .776, 1.179, 1.829}; //Intake, L1, L2, L3, L4

    public static final double[] kHEIGHTS = {.15, .15, 7.86, 25.69, 52.05}; //Intake, L1, L2, L3, L4

    public static final double kELEVATOR_BASE_HEIGHT = .686;

    public static final double kMAX_VOLTS = .4;

    public static final double kPROPORTIONAL_VOLTS = .4;

    public static final double kMAX_VOLT_CHANGE_PER_SECOND = 1;

    public static final double kGRAVITY_VOLTS = .1; // .4 for current coral head
  }

  public static final class NeoMotorConstants {

    //public static final double kFreeSpeedRpm = 5676; REVIEW THIS.
    public static final double kFreeSpeedRpm = 6784;
    public static final int kMaxCurrentDriveMotor = 50;
  }

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
  }
}
