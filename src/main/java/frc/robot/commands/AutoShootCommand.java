package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private boolean end = false;
  private double Velocity;
  private int ShootLevel = 2;
  private ShooterSubsystem shooter;

  public void getShooterLevel(BooleanSupplier L2, BooleanSupplier L3, ShooterSubsystem _shooter) {
    shooter = _shooter;
    if (L2.getAsBoolean()) {
      ShootLevel = 2;
    } else if (L3.getAsBoolean()) {
      ShootLevel = 3;
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Velocity = 0;
    if (ShooterSubsystem.VariablesDefined) {
      double ShootingHeight = 0;
      if (ShootLevel == 2) {
        ShootingHeight = Constants.MIDDLE_HEIGHT - ShooterSubsystem.Bot3d.getZ();
      } else if (ShootLevel == 3) {
        ShootingHeight = Constants.HIGH_HEIGHT - ShooterSubsystem.Bot3d.getZ() - Constants.SHOOTER_FROM_GROUND;
      }
      Velocity = Math.sqrt((-Constants.GRAVITY*(ShooterSubsystem.ClosestDistance*ShooterSubsystem.ClosestDistance))/(2*((Math.cos(Constants.SHOOTER_ANGLE))*(Math.cos(Constants.SHOOTER_ANGLE)))*(ShootingHeight - (ShooterSubsystem.ClosestDistance*Math.tan(Constants.SHOOTER_ANGLE)))));
    } else {
      end = true;
    }
    if (Velocity != 0) {
      shooter.SetShooter(Constants.MOTOR_SPEED_TO_VELOCITY*Velocity);
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.SetShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (end == true) {
      return true;
    }
    return false;
  }
}
