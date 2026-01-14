// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.util.Elastic.ElasticTeleopDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  private SwerveSubsystem swerve;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier angle;
  private BooleanSupplier nitro;
  private BooleanSupplier alignStation;

  private double drivePowerFactor;
  private double rotationalPowerFactor;

  private PIDController anglePid = new PIDController(0, 0, 0);

  private int allianceFactor;


  /** Creates a new Drive. */
  public Drive(
    SwerveSubsystem swerve,
    DoubleSupplier x,
    DoubleSupplier y,
    DoubleSupplier angle,
    BooleanSupplier nitro,
    BooleanSupplier alignStation
  ) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.angle = angle;
    this.nitro = nitro;
    this.alignStation = alignStation;

    rotationalPowerFactor = ElasticTeleopDrive.kROTATIONAL_POWER.get();

    anglePid.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (swerve.isRedAlliance()) {
      allianceFactor = 1;
    } else {
      allianceFactor = -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (nitro.getAsBoolean()) {
      drivePowerFactor = 1;
    } else {
      drivePowerFactor = ElasticTeleopDrive.kDRIVE_POWER.get();
    }

    anglePid.setPID(ElasticTeleopDrive.kP_ANGLE_STATION.get(), ElasticTeleopDrive.kI_ANGLE_STATION.get(), ElasticTeleopDrive.kD_ANGLE_STATION.get());
    
    if (alignStation.getAsBoolean()) {
      swerve.drive(
        new Translation2d(y.getAsDouble(), x.getAsDouble()).times(swerve.getSwerveDrive().getMaximumChassisVelocity() * drivePowerFactor * allianceFactor),
        MathUtil.clamp(
          anglePid.calculate(swerve.getPose().getRotation().getRadians(), swerve.getStationRotation()),
            -ElasticTeleopDrive.kMAX_SPEED_ANGLE_STATION.get(),
            ElasticTeleopDrive.kMAX_SPEED_ANGLE_STATION.get()
          ),
        true
      );
    } else {
      swerve.drive(
        new Translation2d(y.getAsDouble(), x.getAsDouble()).times(swerve.getSwerveDrive().getMaximumChassisVelocity() * drivePowerFactor * allianceFactor),
        -angle.getAsDouble() * rotationalPowerFactor * swerve.getSwerveDrive().getMaximumChassisAngularVelocity(),
        true
      );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
