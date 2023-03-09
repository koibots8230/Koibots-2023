package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  // Auto Shoot Variables


  // Motors/Encoders
  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private RelativeEncoder shooterEncoder;

  private TankDriveSubsystem drive;
  private VisionSubsystem vision;

  public ShooterSubsystem(TankDriveSubsystem _drive, VisionSubsystem _vision) {
    vision = _vision;
    drive = _drive;

    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterEncoder = shooterMotorL.getEncoder();

    shooterMotorL.setIdleMode(IdleMode.kBrake);
    shooterMotorR.setIdleMode(IdleMode.kBrake);
  }

  // ================================Getters================================

  private List<Translation3d> getCubeList(int Level) {
    if (Level == 2) {
      return Constants.MIDDLE_SPOTS;
    } else if (Level == 3) {
      return Constants.HIGH_SPOTS;
    }

    return null;
  }

  public Pose3d getBotPose() {
    Optional<EstimatedRobotPose> photonPose = vision.photonPoseEstimator.update();
    if (photonPose.isPresent()) {
      return photonPose.get().estimatedPose;
    } else {
      return new Pose3d(drive.getOdometryPose());
    }
  }

  /**
   * 
   * @param Robot_Pose
   * @param Level
   * @return
   */
  public Optional<Pose3d> getNearestTarget(Translation3d Robot_Pose, int Level) {
    
    double closestDistance = 100.0;
    double Distance;

    Optional<Pose3d> Closest = Optional.empty();

    for (int a = 0; a < 3; a++) {
      Distance = Robot_Pose.getDistance(getCubeList(Level).get(a));
      if (Distance < closestDistance) {
        closestDistance = Distance;
        Closest = Optional.of(new Pose3d(getCubeList(Level).get(a), new Rotation3d(0, 0, 0)));
      }
    }
    if (closestDistance > Constants.MAX_SHOOTER_RANGE) {
      return Optional.empty();
    }

    return Closest;
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  // ================================Setters================================

  public void SetShooter(double Speed) {
    shooterMotorL.set(Speed);
    shooterMotorR.set(-Speed);
  }

  public void SetShooterVelocity(double velocityMetersPerSecond) {
    shooterMotorL.set(velocityMetersPerSecond*Constants.SHOOTER_VELOCITY_TO_SPEED);
    shooterMotorR.set(-velocityMetersPerSecond*Constants.SHOOTER_VELOCITY_TO_SPEED);
  }


  public class CommunityShotCommand extends CommandBase {
    private ShooterSubsystem shooter;

    public CommunityShotCommand(ShooterSubsystem m_ShooterSubsystem) {
      addRequirements(m_ShooterSubsystem);
      shooter = m_ShooterSubsystem;
    }

    @Override
    public void initialize() {
      shooter.SetShooter(Constants.COMMUNITY_SHOOTER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
      shooter.SetShooter(0);
    }
  }
  
  public class LevelShootCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private int shootLevel;

    public LevelShootCommand(ShooterSubsystem m_ShooterSubsystem, Integer m_shootLevel) {
        addRequirements(m_ShooterSubsystem);
        shooter = m_ShooterSubsystem;
        shootLevel = m_shootLevel;
    }

    public void initialize() {
      if (shootLevel == 2) {
        shooter.SetShooter(Constants.L1_SHOOTER_SPEED);
      } else if (shootLevel == 3) {
        shooter.SetShooter(Constants.L2_SHOOTER_SPEED);
      }
    }

    public void end(boolean interrupted) {
      shooter.SetShooter(0);
    }
  }
   
  public class ShootTimeCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private Timer shootTimer;
    private double shootLimit;
    private boolean hasFinished;
    private double shootSpeed;

    public ShootTimeCommand(ShooterSubsystem subsystem, double time) {
      this(subsystem, time, 0.6);
    }

    public ShootTimeCommand(ShooterSubsystem m_ShooterSubsystem, double ShootTime, double speed) {
        addRequirements(m_ShooterSubsystem);
        shooter = m_ShooterSubsystem;
        shootTimer = new Timer();
        shootLimit = ShootTime;
        hasFinished = false;
       shootSpeed = speed;
    }

    @Override
    public void initialize() {
      shootTimer.start();
      shooter.SetShooter(shootSpeed);
    }

    @Override
    public void execute() {
      // Check for time
      if (shootTimer.get() >= shootLimit) {
        hasFinished = true;
      }
    }

    @Override
    public void end(boolean interrupted) {
      shooter.SetShooter(0);
    }

    @Override 
    public boolean isFinished() {
      return hasFinished;
    }
  }
}