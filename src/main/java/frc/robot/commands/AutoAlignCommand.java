package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class AutoAlignCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private boolean end;
  private Rotation2d  BotRotation;
  private Rotation2d  RotationToTarget;
  private boolean RoationValid;
  private double Rotation;
  private TankDriveSubsystem drive;

  public AutoAlignCommand(TankDriveSubsystem _drive) {
    drive = _drive;
    BotRotation = new Rotation2d(0);
    RotationToTarget = new Rotation2d(0);
    RoationValid = false;
    Rotation = 0;
    end = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ShooterSubsystem.VariablesDefined) {
        BotRotation = ShooterSubsystem.Bot3d.getRotation().toRotation2d();
        RotationToTarget = new Rotation2d(0.0174533*
            (Math.acos(
                (ShooterSubsystem.Bot3d.getX()-ShooterSubsystem.Closest.getX())
            /ShooterSubsystem.ClosestDistance)));
        
        if (RotationToTarget.getDegrees() < 0) { //If on RED side, mabye swap '<' if bot is aligning wrong way
            if (ShooterSubsystem.Bot3d.getY() > ShooterSubsystem.Closest.getY()) { //If target is to the RIGHT
                RotationToTarget = new Rotation2d(0.0174533*(RotationToTarget.getDegrees()+270));
            }
        } else { //If on BLUE side
            if (ShooterSubsystem.Bot3d.getY() > ShooterSubsystem.Closest.getY()) {//If target is to the RIGHT
                RotationToTarget = new Rotation2d(0.0174533*(RotationToTarget.getDegrees()+180));
            } else {
                RotationToTarget = new Rotation2d(0.0174533*(RotationToTarget.getDegrees()+90));
            }
            RoationValid = true;
        }
    } else {
        end = true;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RoationValid) {
        Rotation = BotRotation.getDegrees()-RotationToTarget.getDegrees();
        if (Math.abs(Rotation) < 3) {
            drive.setMotor(0, 0);
            end = true;
        } else if (Rotation < 0) {
            drive.setMotor(Constants.AUTO_SPEED, (Constants.AUTO_SPEED)*-1);
        } else if (Rotation > 0) {
            drive.setMotor((Constants.AUTO_SPEED)*-1, Constants.AUTO_SPEED);
        }
    } else {
        end = true;
    }
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