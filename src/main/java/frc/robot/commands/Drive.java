package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends Command {
  private SwerveSubsystem swerve;
  private DoubleSupplier x;
  private DoubleSupplier y;
  private DoubleSupplier angle;
  private BooleanSupplier nitro;

  private double drivePowerFactor;
  private double rotationalPowerFactor;
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

    rotationalPowerFactor = .5;

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
      drivePowerFactor = .7;
    }
    swerve.drive(
      new Translation2d(y.getAsDouble(), x.getAsDouble()).times(swerve.getSwerveDrive().getMaximumChassisVelocity() * drivePowerFactor * allianceFactor),
      -angle.getAsDouble() * rotationalPowerFactor * swerve.getSwerveDrive().getMaximumChassisAngularVelocity(),
      true
    );
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