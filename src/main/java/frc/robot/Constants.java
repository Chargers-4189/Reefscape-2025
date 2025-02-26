// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static class VisionConstants {

    public static final Transform3d flCamPose = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(6.25),
        Units.inchesToMeters(15.25),
        Units.inchesToMeters(9)
      ),
      new Rotation3d(0, Math.toRadians(0), 0)
    );

    public static final Transform3d frCamPose = new Transform3d(
      new Translation3d(
        Units.inchesToMeters(-6.25),
        Units.inchesToMeters(-15.25),
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
  }

  // Based off a 110lb robot
  // MOI: (1/12)*(weightKG)*(wheelBaseM^2 + trackWidthM^2)
  public static class SwerveConstants {

    public static final double kWheelDiameter = 3; // Inches
    public static final double kWheelRadius =
      Units.inchesToMeters(kWheelDiameter) / 2;
    public static final double kRobotWeight = Units.lbsToKilograms(110);
    public static final double kWheelBase = Units.inchesToMeters(12.018 * 2);
    public static final double kTrackWidth = Units.inchesToMeters(14.508 * 2);
    public static final double kMaxVelocity = 4.92;
    public static final double kWheelCOF = 1.19;
    public static final double kDriveRatio = 5.5;
    public static final double kSteerRatio = 46.423645320197;
    public static final double kMOI =
      (
        (1.0 / 12.0) *
        kRobotWeight *
        (Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2))
      );
    public static final int kDriveAmpLimit = 40;
    public static final int kSteerAmpLimit = 20;
  }

  public static class CoralEffectorConstants {

    public static final double kPLACE_CORAL_ANGLE = -1; // Find the optimal angle for placing coral in absolute encoder
    public static final double kDEFAULT_CORAL_EFFECTER_ANGLE = -1; // Find the optimal default angle in absolute encoder
    public static final int kACTUATOR_ID = 33;
    public static final int kLEFT_MOTOR_ID = 31;
    public static final int kRIGHT_MOTOR_ID = 32;
    public static final int kINTAKE_SENSOR_ID = 34;
    public static final int kOUTTAKE_SENSOR_ID = 35;
    public static final int kMEASURE_THRESHOLD = 5; // milimeters
  }

  public static class ElevatorConstants {

    public static final int kLEFT_MOTOR_ID = 11;
    public static final int kRIGHT_MOTOR_ID = 12;
    public static final int kMIN_DIO_PORT = 0;
    public static final int kMAX_DIO_PORT = 1;
    public static final int kMaxCurrentDriveMotor = 50;

    public static final double kROTATIONS_TO_METERS =
      1.757 * Math.PI * 2.54 / 100;

    // public static final double[] HEIGHTS_METERS = { .720, .700, .776, 1.179,
    // 1.829}; //Intake, L1, L2, L3, L4

    public static final double[] kHEIGHTS = { 0.15, 0.15, 7.9, 26.1, 52.5 }; // Intake, L1, L2, L3, L4

    public static final double kELEVATOR_BASE_HEIGHT = .686;

    public static final double kGRAVITY_VOLTS = .3;
    public static final double kPROPORTIONAL_VOLTS = .8;
    public static final double kMAX_VOLTS = 8;
    public static final double kMAX_VOLT_CHANGE_PER_SECOND = 12;

    public static final double kTOLERANCE = .5;
  }

  public static final class IntakeConstants {

    public static final int kACTUATOR_MOTOR_ID = 41;
    public static final int kDIO_PORT_TOP = 10;
    public static final int kDIO_PORT_BOTTOM = 9;
    public static final double kPOWER_SCALE = .1;

    public static final double kGRAVITY_VOLTS = .01;

    public static final double kUP_ENCODER = 22.5;
    public static final double kMAX_POWER = .5;
    public static final double kPROPORTIONAL_VOLTS = .01;
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
