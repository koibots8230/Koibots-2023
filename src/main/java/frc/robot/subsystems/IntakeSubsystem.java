// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final CANSparkMax midtakeMotor;
    private final RelativeEncoder midtakeEncoder;

    // Boolean for the way that the intake runs. True means forward, false means backwards:
    private boolean runsForward;

    // This motor raises and lowers the intake:
    private final CANSparkMax raiseIntakeMotor;
    private final RelativeEncoder raiseIntakeEncoder;

    private double intakePosition; // This variable refers to the incline of the intake IN DEGREES

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeEncoder = intakeMotor.getEncoder();
        midtakeMotor = new CANSparkMax(Constants.MIDTAKE_MOTOR, MotorType.kBrushless); 
        midtakeMotor.setInverted(false);
        midtakeEncoder = midtakeMotor.getEncoder();

        // raiseIntakeMotor:
        raiseIntakeMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, MotorType.kBrushless);
        raiseIntakeMotor.setInverted(false);
        raiseIntakeEncoder = raiseIntakeMotor.getEncoder();
        raiseIntakeEncoder.setPosition(0);
        intakePosition = 0;

        // Intake starts off going forward:
        runsForward = true;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double inVelocity = intakeEncoder.getVelocity();
        double InCurrent = intakeMotor.getOutputCurrent(); 
        SmartDashboard.putNumber("Intake Motor Speed (RPM)", inVelocity);
        SmartDashboard.putNumber("Main Battery Current (A)", InCurrent);
        double midVelocity = midtakeEncoder.getVelocity();
        double midCurrent = midtakeMotor.getOutputCurrent();
        SmartDashboard.putNumber("Midtake Motor Current (A)", midCurrent);
        SmartDashboard.putNumber("Midtake Motor Speed (RPM)", midVelocity);

    }

    @Override
    public void simulationPeriodic() {
    }

    public void turnOn() {
        intakeMotor.set(Constants.RUNNING_SPEED);
        midtakeMotor.set(Constants.RUNNING_SPEED);
    }

    public void turnOn(Boolean Forwards) {
        if (Forwards){
            intakeMotor.set(Constants.RUNNING_SPEED);
            midtakeMotor.set(Constants.RUNNING_SPEED);
        } else {
            intakeMotor.set(-Constants.RUNNING_SPEED);
            midtakeMotor.set(-Constants.RUNNING_SPEED);
        }
    }

    public void turnOff() {
        intakeMotor.set(0);
        midtakeMotor.set(0);
    }

    public double getRaiseMotorCurrent() {
        return raiseIntakeMotor.getOutputCurrent();
    }

    public double getIntakePosition() {
        return intakePosition;
    }

    public void setRaiseIntakeSpeed(double speed){
        raiseIntakeMotor.set(speed);
    }

    public RelativeEncoder getRaiseEncoder() {
        return raiseIntakeEncoder;
    }

    public CANSparkMax getIntakeRaiseMotor() {
        return raiseIntakeMotor;
    }

    public boolean getForward() {
        return runsForward;
    }

    public void setForward(boolean forward) {
        runsForward = forward;
    }

    public class FlipIntake extends CommandBase {
        IntakeSubsystem m_intake;
        boolean end = false;
        int direction = 1;

        public FlipIntake(IntakeSubsystem subsystem) {
            m_intake = subsystem;
            addRequirements(m_intake);
        }

        @Override
        public void initialize() {
            direction *= -1;
            m_intake.setRaiseIntakeSpeed(direction * Constants.RAISE_SPEED);
        }

        @Override
        public void execute() {
            if (Math.abs(m_intake.getRaiseMotorCurrent()) < Constants.CURRENT_ZONE_AMPS) {
                if (m_intake.getRaiseEncoder().getPosition() > Constants.INTAKE_UP_POSITION || m_intake.getRaiseEncoder().getPosition() < Constants.INTAKE_DOWN_POSITION) {
                end = true;
                } else {
                    return;
                }
            } 
            end = true;
        }

        @Override
        public boolean isFinished() {
            return end;
        }

        @Override
        public void end(boolean interrupted) {
            m_intake.setRaiseIntakeSpeed(0);
        }
    }

   

    public class SwitchIntakeDirection extends CommandBase {
        private final IntakeSubsystem m_IntakeSubsystem;
        public SwitchIntakeDirection(IntakeSubsystem subsystem) {
            m_IntakeSubsystem = subsystem;
            addRequirements(m_IntakeSubsystem); 
        }

        @Override
        public void execute() {
            m_IntakeSubsystem.setForward(!getForward());
            SmartDashboard.putBoolean("Is intake reversed?", m_IntakeSubsystem.getForward());
        }

        @Override 
        public boolean isFinished() {
            return true;
        }
    }
}