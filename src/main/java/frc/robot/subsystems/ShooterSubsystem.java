package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private CANSparkMax shooterMotorL;
  private CANSparkMax shooterMotorR;
  private SparkMaxPIDController leftPID;
  private SparkMaxPIDController rightPID;


  private RelativeEncoder leftShooterEncoder;
  private RelativeEncoder rightShooterEncoder;


  ShooterSubsystem() {
    shooterMotorL = new CANSparkMax(Constants.SHOOTER_MOTOR_L, MotorType.kBrushless);
    shooterMotorR = new CANSparkMax(Constants.SHOOTER_MOTOR_R, MotorType.kBrushless);
    shooterMotorL.setInverted(true);

    leftShooterEncoder = shooterMotorL.getEncoder();
    rightShooterEncoder = shooterMotorR.getEncoder();

    shooterMotorR.setInverted(false);

    leftPID = shooterMotorL.getPIDController();
    rightPID = shooterMotorR.getPIDController();


    leftPID.setP(Constants.SHOOTER_LEFT_P, 0);
    leftPID.setFF(Constants.SHOOTER_LEFT_FF, 0);

    rightPID.setP(Constants.SHOOTER_RIGHT_P, 0);
    rightPID.setFF(Constants.SHOOTER_RIGHT_FF, 0);
  }

  public static ShooterSubsystem get() {
    return m_ShooterSubsystem;
  }

  public void SetShooter(double lSpeed, double rSpeed) {
    SmartDashboard.putNumber("Left Setpoint", lSpeed);
    SmartDashboard.putNumber("Right Setpoint", rSpeed);

    leftPID.setReference(lSpeed, ControlType.kVelocity, 0);
    rightPID.setReference(rSpeed, ControlType.kVelocity, 0);

    SmartDashboard.putNumber("Left Output", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Left Output", rightShooterEncoder.getVelocity());
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
    return new Shoot(0.1);
  }

  public Command L1Shot() {
    return new Shoot(Constants.L1_SHOOTER_SPEED);
  }

  public Command StationToHybridShot() {
    return new Shoot(0.55);
  }

  public Command AutoL2Shot() {
    return new Shoot(Constants.AUTO_L2_SHOOTER_SPEED, Constants.AUTO_L2_SHOOTER_SPEED + 0.2);
  }
  
  public Command L2Shot() {  
    return new Shoot(Constants.L2_SHOOTER_SPEED);
  }

  public Command HybriShot() {
    return new Shoot(Constants.HYBRID_SHOOTER_SPEED);
  }
}