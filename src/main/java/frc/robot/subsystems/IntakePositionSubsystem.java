// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePositionSubsystem extends SubsystemBase {
  private static IntakePositionSubsystem m_IntakePositionSubsystem = new IntakePositionSubsystem();

  private CANSparkMax intakePositionMotor;
  private RelativeEncoder intakePositionEncoder;

  /** Creates a new IntakePositionSubsystem. */
  public IntakePositionSubsystem() {
    intakePositionMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, MotorType.kBrushless);
    intakePositionEncoder = intakePositionMotor.getEncoder();
  }

  public static IntakePositionSubsystem get() {
    return m_IntakePositionSubsystem;
  }

  public void setIntakePositionSpeed(double speed) {
    intakePositionMotor.set(speed);
  }

  public void resetPositionEncoder() {
    intakePositionEncoder.setPosition(0);
  }

  public double getEncoderPosition() {
    return intakePositionEncoder.getPosition();
  }

  public double getMotorCurrent() {
    return intakePositionMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ================================Commands================================ \\

  public class FlipIntake extends CommandBase {
    public FlipIntake() {
      addRequirements(IntakePositionSubsystem.this);
    }

    @Override
    public void initialize() {
      IntakePositionSubsystem.this.setIntakePositionSpeed(Constants.RAISE_SPEED);
      IntakePositionSubsystem.this.resetPositionEncoder();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(IntakePositionSubsystem.this.getEncoderPosition()) > Constants.FLIP_INTAKE_DISTANCE) {
          return true;
        } else if (IntakePositionSubsystem.this.getMotorCurrent() > Constants.CURRENT_CAP) {
          return true;
        }
      return false;
    }

    @Override
    public void end(boolean interrupted) {
        IntakePositionSubsystem.this.setIntakePositionSpeed(0);
    }
  }
}
