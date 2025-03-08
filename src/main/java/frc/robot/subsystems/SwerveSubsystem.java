// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(
    AprilTagFields.k2025ReefscapeWelded
  );

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  /**
   * Swerve drive object.
   */
  File swerveJsonDirectory = new File(
    Filesystem.getDeployDirectory(),
    "swerve"
  );
  SwerveDrive swerveDrive;

  /** Creates a new SwerveDrive. */
  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; // CHANGE TO LOW IN COMP
    try {
      swerveDrive =
        new SwerveParser(swerveJsonDirectory)
          .createSwerveDrive(
            SwerveConstants.kMaxVelocity,
            new Pose2d(2, 7, new Rotation2d())
          );
    } catch (Exception e) {
      System.err.println("Swerve no workie :( " + e.getMessage());
    }

    // ----- PathPlanner ------
    ModuleConfig swerveModuleConfig = new ModuleConfig(
      SwerveConstants.kWheelRadius,
      SwerveConstants.kMaxVelocity,
      SwerveConstants.kWheelCOF,
      DCMotor.getNeoVortex(1),
      SwerveConstants.kDriveRatio,
      SwerveConstants.kDriveAmpLimit,
      1
    );
    RobotConfig config = new RobotConfig(
      SwerveConstants.kRobotWeight,
      SwerveConstants.kMOI,
      swerveModuleConfig,
      this.swerveDrive.swerveDriveConfiguration.moduleLocationsMeters
    );

    AutoBuilder.configure(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {
        return false;
        // Uncomment when using vision or station based control
        // var alliance = DriverStation.getAlliance();
        // if (alliance.isPresent()) {
        // return alliance.get() == DriverStation.Alliance.Red;
        // }
        // return false;
      },
      this
    );
  }
  public Command resetOdometry(Pose2d pose) {
    return Commands.run(
      () -> swerveDrive.resetOdometry(pose)
    );
  }
  public Command resetPosition(Translation2d translation) {
    return resetOdometry(new Pose2d(translation, swerveDrive.getOdometryHeading()));
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
    gyro.reset();
  }

  /*
  public void driveWithAngleSetPoint(double x, double y, double setpoint) {
    double rotationPower =
      (swerveDrive.getOdometryHeading().getDegrees() - setpoint) *
      SwerveConstants.kAlignAngleSpeed;
    rotationPower =
      Math.min(rotationPower, SwerveConstants.kAlignAngleMaxSpeed);
    rotationPower =
      Math.max(rotationPower, -SwerveConstants.kAlignAngleMaxSpeed);
    this.drive(x, y, rotationPower, false);
  }*/

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier headingX,
    DoubleSupplier headingY
  ) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
        new Translation2d(
          translationX.getAsDouble(),
          translationY.getAsDouble()
        ),
        0.8
      );

      swerveDrive.driveFieldOriented(
        swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(),
          scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()
        )
      );
    });
  }

    /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation2d    Rotation setpoint.
   * @return Drive command.
   */
  public void drive(
    double translationX,
    double translationY,
    Rotation2d rotation2d
  ) {
    Translation2d scaledInputs = SwerveMath.scaleTranslation(
      new Translation2d(
        translationX,
        translationY
      ),
      0.8
    );

    swerveDrive.driveFieldOriented(
      swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        rotation2d.getSin(),
        rotation2d.getCos(),
        swerveDrive.getOdometryHeading().getRadians(),
        swerveDrive.getMaximumChassisVelocity()
      )
    );
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier angularRotationX,
    boolean fieldOriented
  ) {
    return run(() -> {
      swerveDrive.drive(
        new Translation2d(
          -translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          -translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        -angularRotationX.getAsDouble() *
        swerveDrive.getMaximumChassisAngularVelocity(),
        fieldOriented,
        false
      );
    });
  }

  public Command xFormation() {
    return run(() -> {
    swerveDrive.lockPose();
    });
  }

  public void drive(
    double translationX,
    double translationY,
    double angularRotationX,
    boolean fieldOriented
  ) {
    swerveDrive.drive(
      new Translation2d(
        -translationX * swerveDrive.getMaximumChassisVelocity(),
        -translationY * swerveDrive.getMaximumChassisVelocity()
      ),
      -angularRotationX * swerveDrive.getMaximumChassisAngularVelocity(),
      fieldOriented,
      false
    );
  }
  public Command driveToPose(Pose2d pose)
  {
    // Create the constraints to use while pathfinding
    //swerveDrive.getMaximumChassisVelocity(), 4.0
    PathConstraints constraints = new PathConstraints(
        1, .5,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }
    public Command driveToAprilTag(int apriltagnumber, Translation2d distanceFromAprilTag, double rotationOffset){
    Pose2d targetAprilTagPose = aprilTagFieldLayout.getTagPose(apriltagnumber).get().toPose2d();
    return this.driveToPose(
      targetAprilTagPose.plus(
        new Transform2d(distanceFromAprilTag,
          new Rotation2d(Units.degreesToRadians(-rotationOffset))
        )
      )
    );
  };
  public Command driveToAprilTag(int aprilTagId, Translation2d distanceFromAprilTag){
    return driveToAprilTag(aprilTagId, distanceFromAprilTag, 180);
  };
  public Command driveToAprilTag(int aprilTagId){
    return driveToAprilTag(aprilTagId,new Translation2d(0,swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters()));

  }
  public Command driveToAprilTag(int aprilTagId, double rotationOffset){
    return driveToAprilTag(aprilTagId, new Translation2d(swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),0), rotationOffset);
  };


  public Command driveToReef(int aprilTagId, boolean right){
    if(right){
      return driveToAprilTag(aprilTagId, new Translation2d(swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),.2));
    }
    return driveToAprilTag(aprilTagId, new Translation2d(swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),-.2));
  };

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(gyro.getAngle());
  }
}
