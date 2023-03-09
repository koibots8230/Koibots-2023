package frc.robot.commands;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShootCommand extends CommandBase {
  /** Creates a new ShootyCommand. */
  private ShooterSubsystem shooter;
  private VisionSubsystem vision;
  private TankDriveSubsystem drive;

  private int level;

  private double distance2d;
  private double heightDifference;

  private double Velocity;

  private double timer = 0;
  private double shooterRunningTimeSeconds = 4; 

  private boolean end = false;

  public AutoShootCommand(ShooterSubsystem _shooter, TankDriveSubsystem _drive, VisionSubsystem _vision, int _level) {
    shooter = _shooter;
    addRequirements(shooter);
    vision = _vision;

    level = _level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose3d botPose = shooter.getBotPose();
    Optional<Pose3d> mabyeTargetPose = shooter.getNearestTarget(botPose.getTranslation(), level);

    if (mabyeTargetPose.isEmpty()) {
      end = true;
    }
    
    Pose3d targetPose = mabyeTargetPose.get();

    distance2d = botPose.getTranslation().toTranslation2d().getDistance(targetPose.getTranslation().toTranslation2d());

    heightDifference = targetPose.getZ() - Constants.SHOOTER_FROM_GROUND - botPose.getZ();

    Velocity = Math.sqrt(
      (-Constants.GRAVITY * Math.pow(distance2d, 2)) / 
      (Math.pow(Math.cos(Constants.SHOOTER_ANGLE), 2) * 
      (heightDifference - (distance2d * Math.tan(Constants.SHOOTER_ANGLE))))
    );

    shooter.SetShooterVelocity(Velocity);

    ShuffleboardTab shootTab = Shuffleboard.getTab("Shooting");

    shootTab.addNumber("Distance To Target", () -> distance2d).withPosition(0, 0).
    withSize(2, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Height Difference With Target", () -> heightDifference).withPosition(0, 1).
    withSize(2, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Calculated Velocity", () -> Velocity).withPosition(0, 2).
    withSize(2, 1).withWidget(BuiltInWidgets.kTextView);
    
    shootTab.addNumber("Target X", () -> targetPose.getX()).withPosition(2, 0).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Target Y", () -> targetPose.getY()).withPosition(2, 1).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Target Z", () -> targetPose.getZ()).withPosition(2, 2).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);

    shootTab.addNumber("Bot X", () -> botPose.getX()).withPosition(3, 0).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Bot Y", () -> botPose.getY()).withPosition(3, 1).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
    shootTab.addNumber("Bot Z", () -> botPose.getZ()).withPosition(3, 2).
    withSize(1, 1).withWidget(BuiltInWidgets.kTextView);
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer == shooterRunningTimeSeconds * 50) {
      end = true;
    } else {
      timer ++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.SetShooterVelocity(0);;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
