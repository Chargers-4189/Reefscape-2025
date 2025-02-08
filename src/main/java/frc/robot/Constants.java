// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  // Based off a 110lb robot
  // MOI: (1/12)*(weightKG)*(wheelBaseM^2 + trackWidthM^2)
  public static class SwerveConstants {
    public static final double kWheelDiameter = 3; // Inches
    public static final double kWheelRadius = Units.inchesToMeters(kWheelDiameter) / 2;
    public static final double kRobotWeight = Units.lbsToKilograms(110);
    public static final double kWheelBase = Units.inchesToMeters(12.018 * 2);
    public static final double kTrackWidth = Units.inchesToMeters(14.508 * 2);
    public static final double kMaxVelocity = 4.92;
    public static final double kWheelCOF = 1.19;
    public static final double kDriveRatio = 5.5;
    public static final double kSteerRatio = 46.423645320197;
    public static final double kMOI = ((1.0/12.0) * kRobotWeight * (Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2)));
    public static final int kDriveAmpLimit = 40;
    public static final int kSteerAmpLimit = 20;
  }


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
