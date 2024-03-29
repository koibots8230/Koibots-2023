package frc.robot.subsystems;


import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    private final CANSparkMax intakeMotor;
    private LinearFilter averager;

    public static IntakeSubsystem get() {
        return m_IntakeSubsystem;
    }

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(10, MotorType.kBrushless);
        intakeMotor.setInverted(true);
        averager = LinearFilter.movingAverage(3);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", averager.calculate(intakeMotor.getOutputCurrent()));
    }

    // ================================Commands================================

    public class RunIntake extends CommandBase {
        public RunIntake() {
            addRequirements(IntakeSubsystem.this);
        }

        @Override
        public void initialize() {
            IntakeSubsystem.this.setIntakeSpeed(Constants.INTAKE_RUNNING_SPEED);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            IntakeSubsystem.this.setIntakeSpeed(0);
        }
    }

    public class RunIntakeReverse extends CommandBase {
        public RunIntakeReverse() {
            addRequirements(IntakeSubsystem.this);
        }

        @Override
        public void initialize() {
            IntakeSubsystem.this.setIntakeSpeed(Constants.INTAKE_REVERSE_SPEED);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            IntakeSubsystem.this.setIntakeSpeed(0);
        }
    }
}