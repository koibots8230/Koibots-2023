package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePositionSubsystem extends SubsystemBase {
  private static final IntakePositionSubsystem intakePositionSubsystem = new IntakePositionSubsystem();
  private final CANSparkMax intakePositionMotor;

  private boolean isUp;

  IntakePositionSubsystem() {
    intakePositionMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, MotorType.kBrushless);
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
    return intakePositionSubsystem;
  }

  public double getMotorCurrent() {
    return intakePositionMotor.getOutputCurrent();
  }

  // ================================Setters================================ \\

  public void setIntakePositionSpeed(double speed) {
    intakePositionMotor.set(speed);
  }

  public void setCoast() {
    intakePositionMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrake() {
    System.out.println("Intake Up/Down Set to Brake");
    intakePositionMotor.setIdleMode(IdleMode.kBrake);
  }

  public void ClearStickies() {
    intakePositionMotor.clearFaults();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // ================================Commands================================ \\

  public class IntakeUpDown extends CommandBase {
    final boolean up;
    boolean end = false;
    final LinearFilter voltageFilter;

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
        System.out.println("VOLTAGE LIMIT HIT");
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
}