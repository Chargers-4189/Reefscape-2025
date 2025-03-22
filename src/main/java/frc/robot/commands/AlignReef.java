// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import frc.util.Stopwatch;
import frc.util.Elastic.ElasticAlign;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignReef extends Command {

  private SwerveSubsystem swerve;
  private boolean alignRight;
  private Pose2d tagPosition;
  //private Pose2d tagGoal;
  //private Pose2d lastPos;
  private double xPower;
  private double yPower;
  private double anglePower;

  private double x;
  private double y;
  private double angle;

  private PIDController xPid = new PIDController(0, 0, 0);
  private PIDController yPid = new PIDController(0, 0, 0);
  private PIDController anglePid = new PIDController(0, 0, 0);

  private double yOffset;

  private Stopwatch stopwatch = new Stopwatch();


  /** Creates a new AutoAlignPose. */
  public AlignReef(SwerveSubsystem swerve, boolean alignRight) {
    this.swerve = swerve;
    this.alignRight = alignRight;
    anglePid.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Align Reef Pose");

    xPid.setPID(ElasticAlign.kP_X.get(), ElasticAlign.kI_X.get(), ElasticAlign.kD_X.get());
    yPid.setPID(ElasticAlign.kP_Y.get(), ElasticAlign.kI_Y.get(), ElasticAlign.kD_Y.get());
    anglePid.setPID(ElasticAlign.kP_ANGLE.get(), ElasticAlign.kI_ANGLE.get(), ElasticAlign.kD_ANGLE.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    
    if (alignRight) {
      if (Cameras.LEFT_CAM.getEstimateTagPose() != null) {
        tagPose = Cameras.LEFT_CAM.getEstimateTagPose();
      } else {
        System.out.println("null");
      }
    } else {
      if (Cameras.RIGHT_CAM.getEstimateTagPose() != null) {
        tagPose = Cameras.RIGHT_CAM.getEstimateTagPose();
      } else {
        System.out.println("null");
      }
    }
    x = tagPose.getX();
    y = tagPose.getY();
    angle = -tagPose.getRotation().plus(new Rotation3d(0, 0, Math.PI)).getZ();
    */

    if (alignRight) {
      yOffset = ElasticAlign.kDIST_OFFSET_RIGHT.get();
    } else {
      yOffset = ElasticAlign.kDIST_OFFSET_LEFT.get();
    }

    tagPosition = swerve.getClosestReefTagPose().relativeTo(swerve.getPose());

    x = tagPosition.getX();
    y = tagPosition.getY() + yOffset;
    angle = tagPosition.getRotation().getRadians();

    xPower = xPid.calculate(x, ElasticAlign.kDIST_FROM_REEF.get());
    yPower = yPid.calculate(y, 0);
    anglePower = anglePid.calculate(angle, Math.PI);

    
    System.out.println("X: " + x + " Y: " + y + " Angle: " + angle +
    " PowerX: " + xPower + " PowerY: " + yPower + " PowerAngle: " + anglePower);
    swerve.drive(
      new Translation2d(
        MathUtil.clamp(
          xPower,
          -ElasticAlign.kMAX_SPEED_X.get(),
          ElasticAlign.kMAX_SPEED_X.get()
        ),
        MathUtil.clamp(
          yPower,
          -ElasticAlign.kMAX_SPEED_Y.get(),
          ElasticAlign.kMAX_SPEED_Y.get()
        )
      ),
      MathUtil.clamp(
        anglePower,
        -ElasticAlign.kMAX_SPEED_ANGLE.get(),
        ElasticAlign.kMAX_SPEED_ANGLE.get()
      ),
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false);
    stopwatch.reset();
  }

  // Returns true when the command should end.P
  @Override
  public boolean isFinished() {
    if (stopwatch.hasStarted()) {
      System.out.println("started");
      return stopwatch.hasTriggered();
    } else if (
      (Math.abs(x - ElasticAlign.kDIST_FROM_REEF.get()) < ElasticAlign.kX_TOLERANCE.get()) && 
      (Math.abs(y - 0) < ElasticAlign.kY_TOLERANCE.get()) && 
      (Math.abs(angle - Math.PI) < ElasticAlign.kANGLE_TOLERANCE.get())
    ) {
      stopwatch.start(ElasticAlign.kEXTRA_ALIGNMENT_TIME.get());
    }
    return false;
  }
}
