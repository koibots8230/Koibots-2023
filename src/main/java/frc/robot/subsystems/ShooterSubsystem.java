package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  // Motors/Encoders
  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;

  private RelativeEncoder shooterEncoder;

  public ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterMotorL.setInverted(true);

    shooterEncoder = shooterMotorL.getEncoder();

    shooterMotorR.follow(shooterMotorL, true);
  }

  // ================================Getters================================ \\

  public static ShooterSubsystem get() {
    return m_ShooterSubsystem;
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  private List<Translation3d> getCubeList(int Level) {
    if (Level == 1) {
      return Constants.MIDDLE_SPOTS;
    } else if (Level == 2) {
      return Constants.HIGH_SPOTS;
    }

    return null;
  }

  public Pose3d getBotPose() {
    Optional<EstimatedRobotPose> photonPose = VisionSubsystem.get().photonPoseEstimator.update();
    if (photonPose.isPresent()) {
      return photonPose.get().estimatedPose;
    } else {
      return new Pose3d(TankDriveSubsystem.get().getRobotPose());
    }
  }

  public Optional<Pose3d> getNearestTarget(Translation3d Robot_Pose, int Level) {

    double closestDistance = 100.0;
    double Distance;

    Optional<Pose3d> Closest = Optional.empty();

    for (int a = 0; a < 3; a++) {
      Distance = Robot_Pose.toTranslation2d().getDistance(getCubeList(Level).get(a).toTranslation2d());
      if (Distance < closestDistance) {
        closestDistance = Distance;
        Closest = Optional.of(new Pose3d(getCubeList(Level).get(a), new Rotation3d()));
      }
    }
    if (closestDistance > Constants.MAX_SHOOTER_RANGE) {
      return Optional.empty();
    }

    return Closest;
  }

  // ================================Setters================================ \\

  public void SetShooter(double speed) {
    shooterMotorL.set(speed);
  }

  public void SetShooterVelocity(double velocityMetersPerSecond) {
    shooterMotorL.set(velocityMetersPerSecond * Constants.SHOOTER_VELOCITY_TO_SPEED);
  }

  // ================================Commands================================ \\

  public class Shoot extends CommandBase {
    double m_speed;

    public Shoot(double speed) {
      this.m_speed = speed;
      addRequirements(ShooterSubsystem.get());
    }

    @Override
    public void initialize() {
      ShooterSubsystem.this.SetShooter(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.this.SetShooter(0);
    }
  }

  public Command CommunityShot() {
    return new Shoot(Constants.COMMUNITY_SHOOTER_SPEED);
  }

  public Command L1Shot() {
    return new Shoot(Constants.L1_SHOOTER_SPEED);
  }

  public Command L2Shot() {
    return new Shoot(Constants.L2_SHOOTER_SPEED);
  }
}