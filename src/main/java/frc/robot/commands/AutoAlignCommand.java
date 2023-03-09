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


  public AutoAlignCommand(TankDriveSubsystem _drive, ShooterSubsystem _shooter) {
    drive = _drive;
    shooter = _shooter;

    addRequirements(drive);
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
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotationToTarget = getRotationToTarget(botPose, targetPose);
    if (Math.abs(rotationToTarget) < 3){
      end = true;
    } else if (rotationToTarget > 0) {
      drive.setMotor(-.5, .5);
    } else if (rotationToTarget < 0) {
      drive.setMotor(.5, -.5);
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