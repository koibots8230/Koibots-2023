package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  // Motors/Encoders
  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private CANSparkMax starWheelMotor;

  private RelativeEncoder shooterEncoder;

  public ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    starWheelMotor = new CANSparkMax(Constants.STAR_WHEELS_MOTOR_L, MotorType.kBrushless);
    shooterEncoder = shooterMotorL.getEncoder();

    shooterMotorR.follow(shooterMotorL);
  }

  public static ShooterSubsystem get() {
    return m_ShooterSubsystem;
  }

  public double getShooterSpeed() {
    return shooterEncoder.getVelocity();
  }
  
  public void SetShooter(double speed) {
    shooterMotorL.set(speed);
  }

  public void SetStarWheels(double speed) {
    starWheelMotor.set(speed);
  }
  
  public class Shoot extends CommandBase {
    double m_speed;
    public Shoot(double speed) {
      this.m_speed = speed;
    }

    @Override
    public void initialize() {
      ShooterSubsystem.this.SetShooter(m_speed);
      ShooterSubsystem.this.SetStarWheels(Constants.STARS_RUNNING_SPEED);
    }

    @Override
    public void execute() {
      if (!IndexerSubsystem.get().isIndexerFilled()) {
        ShooterSubsystem.this.SetStarWheels(0);
      }
    }

    @Override
    public void end(boolean interrupted) {
      ShooterSubsystem.this.SetShooter(0);
      ShooterSubsystem.this.SetStarWheels(0);
    }
  }

  public Shoot HybridShot() {
    return new Shoot(Constants.COMMUNITY_SHOOTER_SPEED);
  }

  public Shoot L1Shot() {
    return new Shoot(Constants.L1_SHOOTER_SPEED);
  }

  public Shoot L2Shot() {
    return new Shoot(Constants.L2_SHOOTER_SPEED);
  }
}