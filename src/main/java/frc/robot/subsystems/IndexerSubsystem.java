// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  private CANSparkMax IndexerMotor;
  private AnalogInput BeamBreakSensor;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    IndexerMotor = new CANSparkMax(Constants.MIDTAKE_MOTOR, MotorType.kBrushless);
    BeamBreakSensor = new AnalogInput(Constants.BEAM_BREAK);
  }

  public void setIndexerSpeed(double speed) {
    IndexerMotor.set(speed);
  }

  public boolean isIndexerFilled() {
      if (BeamBreakSensor.getVoltage() > Constants.SENSOR_TRIGGERED) {
        return true;
      }
      return false;
  }

  public static IndexerSubsystem getIndexerSubsystem() {
    return m_IndexerSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public class RunIndexer extends CommandBase {
    public RunIndexer() {
      addRequirements(IndexerSubsystem.this);
    }

    @Override
    public void initialize() {
      IndexerSubsystem.this.setIndexerSpeed(Constants.BELT_RUNNING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return IndexerSubsystem.this.isIndexerFilled();
    }

    @Override
    public void end(boolean interrupted) {
        IndexerSubsystem.this.setIndexerSpeed(0);
    }
  }

  public class RunIndexerReverse extends CommandBase {
    public RunIndexerReverse() {
      addRequirements(IndexerSubsystem.this);
    }

    @Override
    public void initialize() {
      IndexerSubsystem.this.setIndexerSpeed(-Constants.BELT_RUNNING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        IndexerSubsystem.this.setIndexerSpeed(0);
    }
  }
}