package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePositionSubsystem extends SubsystemBase {
  private static IntakePositionSubsystem m_IntakePositionSubsystem = new IntakePositionSubsystem();
  private int m_direction = 1;
  private CANSparkMax intakePositionMotor;
  private RelativeEncoder intakePositionEncoder;

  private boolean isUp;

  public IntakePositionSubsystem() {
    intakePositionMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, MotorType.kBrushless);
    intakePositionEncoder = intakePositionMotor.getEncoder();
    isUp = true;
  }

  // ================================Getters================================ \\

  public void switchIntakeState() {
    if (isUp) {
      isUp = false;
      intakePositionMotor.setIdleMode(IdleMode.kCoast);
    }
    isUp = true;
    intakePositionMotor.setIdleMode(IdleMode.kBrake);
  }

  public static IntakePositionSubsystem get() {
    return m_IntakePositionSubsystem;
  }

  public double getEncoderPosition() {
    return intakePositionEncoder.getPosition();
  }

  public double getMotorCurrent() {
    return intakePositionMotor.getOutputCurrent();
  }

  public int getDirection() {
    m_direction *= -1;
    return m_direction;
  }

  // ================================Setters================================ \\

  public void setIntakePositionSpeed(double speed) {
    intakePositionMotor.set(speed);
  }

  public void resetPositionEncoder() {
    intakePositionEncoder.setPosition(0);
  }

  public void setCoast() {
    intakePositionMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    intakePositionMotor.setIdleMode(IdleMode.kBrake);
  }

  public void ClearStickies() {
    intakePositionMotor.clearFaults();
}

  // ================================Commands================================ \\

  public class IntakeUpDown extends CommandBase {
    boolean up;
    boolean end = false;
    LinearFilter voltageFilter;

    public IntakeUpDown(boolean _up) {
      up = _up;
      voltageFilter = LinearFilter.movingAverage(5);
      addRequirements(IntakePositionSubsystem.get());
    }

    @Override
    public void initialize() {
      IntakePositionSubsystem.this.switchIntakeState();
      if (up) {
        IntakePositionSubsystem.get().setIntakePositionSpeed(.3);
      } else {
        IntakePositionSubsystem.get().setIntakePositionSpeed(-.3);
      }
    }

    @Override
    public void execute() {
        if (voltageFilter.calculate(Math.abs(IntakePositionSubsystem.get().getMotorCurrent())) > Constants.CURRENT_CAP) {
          end = true;
        }
    }

    @Override
    public boolean isFinished() {
      return end;
    }

    @Override
    public void end(boolean interrupted) {
      IntakePositionSubsystem.get().setIntakePositionSpeed(0);
    }

  }

  public class FlipIntake extends CommandBase {
    public FlipIntake() {
      addRequirements(IntakePositionSubsystem.this);
    }

    @Override
    public void initialize() {
      int direction = IntakePositionSubsystem.this.getDirection();
      IntakePositionSubsystem.this.setIntakePositionSpeed(Constants.RAISE_SPEED * direction);
      if (direction < 0) {
        IntakePositionSubsystem.this.setCoast();
      } else {
        IntakePositionSubsystem.this.setBrake();
      }
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
        IntakePositionSubsystem.this.switchIntakeState();
    }
  }
}