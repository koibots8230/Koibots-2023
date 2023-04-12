package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private static final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final CANSparkMax indexerMotor;
  private final AnalogInput breamBreak;
  private Boolean useBeamBreak = true;
  private final LinearFilter indexerAverageCurrent;

  IndexerSubsystem() {
    indexerMotor = new CANSparkMax(Constants.MIDTAKE_MOTOR, MotorType.kBrushless);
    breamBreak = new AnalogInput(Constants.BEAM_BREAK);
    indexerAverageCurrent = LinearFilter.movingAverage(3);
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Use Beam Break", this.useBeamBreak);
      SmartDashboard.putNumber("Beam Break", breamBreak.getVoltage());
      SmartDashboard.putBoolean("Beam break triggered", breamBreak.getVoltage() < Constants.SENSOR_TRIGGERED);
      SmartDashboard.putNumber("Indexer Current", indexerAverageCurrent.calculate(indexerMotor.getOutputCurrent()));
      SmartDashboard.putNumber("Output Current", indexerMotor.getOutputCurrent());
  }

  public void setIndexerSpeed(double speed) {
    indexerMotor.set(speed);
  }

  public AnalogInput getBeamBreak() {
    return breamBreak;
  }

  public boolean getUseBeamBreak() {
    return useBeamBreak;
  }

  public void changeUseBeamBreak() {
    useBeamBreak = ! useBeamBreak;
  }

  public static IndexerSubsystem get() {
    return indexerSubsystem;
  }
    // ================================Commands================================ \\

  public class RunUntilBeam extends CommandBase{
    final AnalogInput beamBreak;
    double count;

    public RunUntilBeam() {
      addRequirements(IndexerSubsystem.this);
      beamBreak = IndexerSubsystem.get().getBeamBreak();
    }

    @Override
    public void initialize() {
      IndexerSubsystem.this.setIndexerSpeed(Constants.BELT_RUNNING_SPEED);
      count = 0;
    }

    @Override
    public void execute() {
      if (beamBreak.getVoltage() < Constants.SENSOR_TRIGGERED) {
        count ++;
      } else {
        count = 0;
      }
    }

    @Override
    public boolean isFinished() {
        return count >= 2 && IndexerSubsystem.this.getUseBeamBreak();
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
      IndexerSubsystem.this.setIndexerSpeed(Constants.BELT_REVERSE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        IndexerSubsystem.this.setIndexerSpeed(0);
    }
  }
}