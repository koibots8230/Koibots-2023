package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;

  private RelativeEncoder shooterEncoder;

  ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterMotorL.setInverted(true);

    shooterEncoder = shooterMotorL.getEncoder();

    shooterMotorR.setInverted(false);
  }

  public static ShooterSubsystem get() {
    return m_ShooterSubsystem;
  }
   
  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }

  public void SetShooter(double lSpeed, double rSpeed) {
    shooterMotorL.set(lSpeed);
    shooterMotorR.set(rSpeed);
  }

  // ================================Commands================================ \\
  
  public class Shoot extends CommandBase {
    double leftSpeed;
    double rightSpeed;
    
    public Shoot(double speed) {
      this.leftSpeed = speed;
      this.rightSpeed = speed;
      addRequirements(ShooterSubsystem.get());
    }

    public Shoot(double lSpeed, double rSpeed) {
      this.leftSpeed = lSpeed;
      this.rightSpeed = rSpeed;
      addRequirements(ShooterSubsystem.get());
    }

    @Override
    public void initialize() {
      ShooterSubsystem.this.SetShooter(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.this.SetShooter(0, 0);
    }
  }

  public Command CommunityShot() {
    return new Shoot(Constants.COMMUNITY_SHOOTER_SPEED, Constants.COMMUNITY_SHOOTER_SPEED);
  }

  public Command L1Shot() {
    return new Shoot(Constants.L1_SHOOTER_SPEED, Constants.L1_SHOOTER_SPEED*1.5);
  }

  public Command StationToHybridShot() {
    return new Shoot(0.55, 0.55*1.5);
  }

  public Command AutoL2Shot() {
    return new Shoot(Constants.AUTO_L2_SHOOTER_SPEED, Constants.AUTO_L2_SHOOTER_SPEED*1.5);
  }
  
  public Command L2Shot() {  
    return new Shoot(Constants.L2_SHOOTER_SPEED, Constants.L2_SHOOTER_SPEED*1.5);
  }

  public Command HybriShot() {
    return new Shoot(Constants.HYBRID_SHOOTER_SPEED, Constants.HYBRID_SHOOTER_SPEED*1.5);
  }
}