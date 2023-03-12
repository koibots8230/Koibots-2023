package frc.robot.subsystems;

import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  // Auto Shoot Variables
  public static Pose3d Bot3d = null;
  public static boolean VariablesDefined = false;
  public static Translation3d Closest = new Translation3d(0, 0, 0);
  public static double ClosestDistance = 0;
  private int count = 0;
  private int ShootLevel = 2;
  private double xDistance;
  private double yDistance;
  private double distance;
  private Translation2d Spot = new Translation2d(0, 0);

  // Motors/Encoders
  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private RelativeEncoder shooterEncoder;

  public ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterEncoder = shooterMotorL.getEncoder();

    shooterMotorL.setIdleMode(IdleMode.kBrake);
    shooterMotorR.setIdleMode(IdleMode.kBrake);
  }

  public static ShooterSubsystem get() {
    return m_ShooterSubsystem;
  }

  private List<Translation3d> getCubeList(int Level) {
    if (Level == 2) {
      return Constants.MIDDLE_SPOTS;
    } else if (Level == 3) {
      return Constants.HIGH_SPOTS;
    }

    return null;
  }

  public Pose3d getNearestTarget(Translation3d Robot_Pose, int Level) {
    
    Translation3d Closest = new Translation3d(0, 0, 0);
    double closestDistance = 0;
    double Distance;
    
    for (int a = 0; a < 3; a++) {
      Distance = Robot_Pose.getDistance(getCubeList(Level).get(a));
      if (Distance > closestDistance) {
        closestDistance = Distance;
        Closest = getCubeList(Level).get(a);
      }
    }

    if (ClosestDistance > Constants.MAX_SHOOTER_RANGE) {
      return new Pose3d(0, 0, 0, new Rotation3d());
    }
    return new Pose3d(Closest, new Rotation3d());
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }
  
  public void SetShooter(double Speed) {
    shooterMotorR.set(-Speed);
    shooterMotorL.set(Speed);
  }
  
  public Pose3d g54jetPose() {
    return Bot3d;
  }

  public class CommunityShotCommand extends CommandBase {

    public CommunityShotCommand() {
      addRequirements(ShooterSubsystem.this);
    }

    @Override
    public void initialize() {
      ShooterSubsystem.this.SetShooter(Constants.COMMUNITY_SHOOTER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.this.SetShooter(0);
    }
  }
  
  public class LevelShootCommand extends CommandBase {
    private int shootLevel;

    public LevelShootCommand(Integer m_shootLevel) {
        addRequirements(ShooterSubsystem.this);
        shootLevel = m_shootLevel;
    }

    public void initialize() {
      if (shootLevel == 2) {
        ShooterSubsystem.this.SetShooter(Constants.L1_SHOOTER_SPEED);
      } else if (shootLevel == 3) {
        ShooterSubsystem.this.SetShooter(Constants.L2_SHOOTER_SPEED);
      }
    }

    public void end(boolean interrupted) {
      ShooterSubsystem.this.SetShooter(0);
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