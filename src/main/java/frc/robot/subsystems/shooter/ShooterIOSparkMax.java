package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO{
    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    protected ShooterIOSparkMax() {
        leftShooter = new CANSparkMax(Constants.SHOOTER_MOTOR_L, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightShooter = new CANSparkMax(Constants.SHOOTER_MOTOR_R, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = leftShooter.getEncoder();
        rightEncoder = rightShooter.getEncoder();

        leftShooter.setInverted(true);
    }

    @Override
    public void updateState(ShooterIOState state) {
        state.leftRpm = leftEncoder.getVelocity();
        state.rightRpm = rightEncoder.getVelocity();
    }

    @Override
    public void setVoltages(double left, double right) {
        leftShooter.setVoltage(left);
        rightShooter.setVoltage(right);
    }

    @Override
    public void stop() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }
}
