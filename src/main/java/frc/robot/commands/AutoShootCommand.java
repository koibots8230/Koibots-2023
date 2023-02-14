package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private boolean end = false;
  public double Velocity;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ShootLevel = 2;
    Velocity = 0;
    if (ShooterSubsystem.VariablesDefined) {
      double ShootingHeight = 0;
      if (ShootLevel == 2) {
        ShootingHeight = Constants.MIDDLE_HEIGHT - ShooterSubsystem.Bot3d.getZ();
      } else {
        ShootingHeight = Constants.HIGH_HEIGHT - ShooterSubsystem.Bot3d.getZ() - Constants.SHOOTER_FROM_GROUND;
      }
      Velocity = Math.sqrt((-Constants.GRAVITY*(ShooterSubsystem.ClosestDistance*ShooterSubsystem.ClosestDistance))/(2*((Math.cos(Constants.SHOOTER_ANGLE))*(Math.cos(Constants.SHOOTER_ANGLE)))*(ShootingHeight - (ShooterSubsystem.ClosestDistance*Math.tan(Constants.SHOOTER_ANGLE)))));
    } else {
      end = true;
    }
    if (Velocity != 0) {
      ShooterSubsystem.SetShooter(Constants.MOTOR_SPEED_TO_VELOCITY*Velocity);
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
