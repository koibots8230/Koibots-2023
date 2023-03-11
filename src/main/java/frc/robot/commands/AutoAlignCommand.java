package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoAlignCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private TankDriveSubsystem drive;
  private ShooterSubsystem shooter;

  private Pose3d botPose;
  private Pose3d targetPose;
  private double rotationToTarget;

  private boolean end = false;

  private double angleDeadzone;


  public AutoAlignCommand(TankDriveSubsystem _drive, ShooterSubsystem _shooter) {
    drive = _drive;
    shooter = _shooter;

    addRequirements(drive);
  }

  public double getAngleDeadzone(Pose3d botPose, Pose3d targetPose) {
    double angle1 = Math.atan(
      (botPose.getX()-targetPose.getX()) / 
      (Math.abs(botPose.getY()-targetPose.getY()) + 
      (Constants.AVAILABLE_CUBE_NODE_SPACE/2))
    );
    double angle2 = Math.atan(
      Math.abs(botPose.getY()-targetPose.getY()) / 
      (botPose.getX()-targetPose.getX())
    );
    return 90 - (angle1 + angle2);
  }

  public double getRotationToTarget(Pose3d botPose, Pose3d targetPose) {
    return Math.atan(
      (botPose.getX()-targetPose.getX()) / 
      (botPose.getY() - targetPose.getY())
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    botPose = shooter.getBotPose();

    Optional<Pose3d> mabyeTargetPose = shooter.getNearestTarget(botPose.getTranslation(), 0);

    if (mabyeTargetPose.isEmpty()) {
      end = true;
    }
    targetPose = mabyeTargetPose.get();

    angleDeadzone = getAngleDeadzone(botPose, targetPose);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationToTarget = getRotationToTarget(botPose, targetPose);
    if (Math.abs(rotationToTarget) < angleDeadzone){
      end = true;
    } else if (rotationToTarget > 0) {
      drive.setMotor(Constants.ALIGN_DRIVE_SPEED, Constants.ALIGN_DRIVE_SPEED);
    } else if (rotationToTarget < 0) {
      drive.setMotor(Constants.ALIGN_DRIVE_SPEED, Constants.ALIGN_DRIVE_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotor(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}