package frc.robot.subsystems;

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
    shooterMotorR.setInverted(true);
    shooterMotorR.follow(shooterMotorL);

    shooterEncoder = shooterMotorL.getEncoder();
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }
  
  public void SetShooter(double Speed) {
    shooterMotorL.set(-Speed);
  }
  
  public Pose3d getPose() {
    return Bot3d;
  }

  public class CommunityShotCommand extends CommandBase {
    private DoubleSupplier m_Trigger;

    public CommunityShotCommand(ShooterSubsystem m_ShooterSubsystem) {
      addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
      SetShooter(.75);
    }

    @Override
    public void end(boolean interrupted) {
      SetShooter(0);
    }
}

  @Override
  public void simulationPeriodic() {

  }
  

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
}