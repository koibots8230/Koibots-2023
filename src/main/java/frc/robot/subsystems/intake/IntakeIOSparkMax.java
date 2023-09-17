package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IntakeIOSparkMax implements IntakeIO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final CANSparkMax positionMotor;

    protected IntakeIOSparkMax() {
        motor = new CANSparkMax(Constants.INTAKE_MOTOR, kBrushless);
        encoder = motor.getEncoder();

        motor.setInverted(true);

        positionMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, kBrushless);
    }

    @Override
    public void updateState(IntakeIOState state) {
        state.intakeRpm = encoder.getVelocity();
        state.intakePositionCurrent = positionMotor.getOutputCurrent();
    }

    @Override
    public void clearPositionStickies() {
        positionMotor.clearFaults();
    }

    @Override
    public void run() {
        motor.setVoltage(Constants.INTAKE_RUNNING_SPEED);
    }

    @Override
    public void runReversed() {
        motor.setVoltage(Constants.INTAKE_REVERSE_SPEED);
    }

    @Override
    public void setPositionBrake() {
        positionMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void setPositionCoast() {
        positionMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setIntakePositionSpeed(double speed) {
        positionMotor.setVoltage(speed);
    }

    @Override
    public void stopIntakePosition() {
        positionMotor.stopMotor();
    }
}
