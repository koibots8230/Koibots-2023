package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  private CANSparkMax IndexerMotor;
  private AnalogInput m_breamBreak;
  private Boolean useBeamBreak = true;

  public IndexerSubsystem() {
    IndexerMotor = new CANSparkMax(Constants.MIDTAKE_MOTOR, MotorType.kBrushless);
    m_breamBreak = new AnalogInput(Constants.BEAM_BREAK);
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Use Beam Break", this.useBeamBreak);
      SmartDashboard.putNumber("Beam Break", m_breamBreak.getVoltage());
      SmartDashboard.putBoolean("Beam break triggered", this.isIndexerFilled());
  }

  public void setIndexerSpeed(double speed) {
    IndexerMotor.set(speed);
  }

  public boolean isIndexerFilled() {
    return m_breamBreak.getVoltage() < Constants.SENSOR_TRIGGERED;
  }

  public boolean getUseBeamBreak() {
    return useBeamBreak;
  }

  public void changeUseBeamBreak() {
    useBeamBreak = ! useBeamBreak;
  }

  public static IndexerSubsystem get() {
    return m_IndexerSubsystem;
  }
    // ================================Commands================================ \\

  public class RunUntilBeam extends CommandBase{
    public RunUntilBeam() {
      addRequirements(IndexerSubsystem.this);
    }

        @Override
    public void initialize() {
      IndexerSubsystem.this.setIndexerSpeed(Constants.BELT_RUNNING_SPEED);
    }

    @Override
    public boolean isFinished() {
        return IndexerSubsystem.this.isIndexerFilled() && IndexerSubsystem.this.getUseBeamBreak();
    }

    @Override
    public void end(boolean interrupted) {
        IndexerSubsystem.this.setIndexerSpeed(0);
    }
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
        return false;
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