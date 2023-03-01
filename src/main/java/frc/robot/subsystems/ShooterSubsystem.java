package frc.robot.subsystems;

import java.lang.System.Logger.Level;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
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

  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private RelativeEncoder shooterEncoder;

  public ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterEncoder = shooterMotorL.getEncoder();
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }
  
  public void SetShooter(double Speed) {
    shooterMotorR.set(Speed);
    shooterMotorL.set(Speed);
  }
  
  public Pose3d getPose() {
    return Bot3d;
  }

  public class CommunityShotCommand extends CommandBase {
    private DoubleSupplier m_Trigger;
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
        shooter.SetShooter(Constants.L2_SHOOTER_SPEED);
      } else if (shootLevel == 3) {
        shooter.SetShooter(Constants.L3_SHOOTER_SPEED);
      }
    }

    public void end(boolean interrupted) {
      shooter.SetShooter(0);
    }
  }

  @Override
  public void simulationPeriodic() {

  }
  

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
}