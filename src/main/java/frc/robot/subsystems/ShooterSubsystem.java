package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final CANSparkMax shooterMotorL;
  private final CANSparkMax shooterMotorR;

  private final RelativeEncoder leftShooterEncoder;
  private final RelativeEncoder rightShootEncoder;

  ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterMotorL.setInverted(true);

    leftShooterEncoder = shooterMotorL.getEncoder();
    rightShootEncoder = shooterMotorR.getEncoder();

    shooterMotorR.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Speed", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Right Speed", rightShootEncoder.getVelocity());
  }

  public static ShooterSubsystem get() {
    return shooterSubsystem;
  }

  public void SetShooter(double lSpeed, double rSpeed) {
    shooterMotorL.set(lSpeed);
    shooterMotorR.set(rSpeed);
  }

  // ================================Commands================================ \\
  
  public class Shoot extends CommandBase {
    final double leftSpeed;
    final double rightSpeed;
    
    public Shoot(double speed) {
      this.leftSpeed = speed;
      this.rightSpeed = speed;
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
    return new Shoot(Constants.COMMUNITY_SHOOTER_SPEED);
  }

  public Command L1Shot() {
    return new Shoot(Constants.L1_SHOOTER_SPEED);
  }

  public Command StationToHybridShot() {
    return new Shoot(0.6);
  }

  public Command AutoL2Shot() {
    return new Shoot(Constants.AUTO_L2_SHOOTER_SPEED);
  }
  
  public Command L2Shot() {  
    return new Shoot(Constants.L2_SHOOTER_SPEED);
  }

  public Command HybridShot() {
    return new Shoot(Constants.HYBRID_SHOOTER_SPEED);
  }
}