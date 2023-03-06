package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.apache.commons.cli.HelpFormatter;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private ShooterSubsystem shooter;
  private VisionSubsystem vision;
  private int level;
  private Translation3d target;

  private double distance2d;
  private double hightDifference;

  private double Velocity;

  private boolean end = false;

  public AutoShootCommand(ShooterSubsystem _shooter, VisionSubsystem _vision, int _level) {
    shooter = _shooter;
    addRequirements(shooter);
    vision = _vision;

    level = _level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Optional<EstimatedRobotPose> photonPose = vision.photonPoseEstimator.update();

    if  (!photonPose.isPresent()) {
      end = true;
    }
    Pose3d botPose = photonPose.get().estimatedPose;
    Pose3d targetPose = shooter.getNearestTarget(botPose.getTranslation(), level);

    distance2d = Math.sqrt(
      ((botPose.getX() - targetPose.getX()) * (botPose.getX() - targetPose.getX())) +
      ((botPose.getY() - targetPose.getY()) * (botPose.getY() - targetPose.getY())) 
    );

    hightDifference = targetPose.getZ() - Constants.SHOOTER_FROM_GROUND - botPose.getZ();

    Velocity = 0;

    shooter.SetShooter(Velocity*Constants.VELOCITY_TO_SPEED);

    // Velocity = 0;
    // if (ShooterSubsystem.VariablesDefined) {
    //   double ShootingHeight = 0;
    //   if (ShootLevel == 2) {
    //     ShootingHeight = Constants.MIDDLE_HEIGHT - ShooterSubsystem.Bot3d.getZ();
    //   } else if (ShootLevel == 3) {
    //     ShootingHeight = Constants.HIGH_HEIGHT - ShooterSubsystem.Bot3d.getZ() - Constants.SHOOTER_FROM_GROUND;
    //   }
    //   Velocity = Math.sqrt((-Constants.GRAVITY*(ShooterSubsystem.ClosestDistance*ShooterSubsystem.ClosestDistance))/(2*((Math.cos(Constants.SHOOTER_ANGLE))*(Math.cos(Constants.SHOOTER_ANGLE)))*(ShootingHeight - (ShooterSubsystem.ClosestDistance*Math.tan(Constants.SHOOTER_ANGLE)))));
    // } else {
    //   end = true;
    // }
    // if (Velocity != 0) {
    //   shooter.SetShooter(Constants.MOTOR_SPEED_TO_VELOCITY*Velocity);
    // }
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
