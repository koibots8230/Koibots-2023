package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Drive extends SubsystemBase {
    private static final Drive instance = new Drive();
    private final DriveIO io;
    private final DriveIOStateAutoLogged state = new DriveIOStateAutoLogged();
    private double speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;

    public static Drive get() {
        return instance;
    }

    private Drive() {
        if (Robot.isReal()) {
            io = new DriveIOSparkMax();
        } else {
            io = null; // TODO: Create sim version
        }
    }

    @Override
    public void periodic() {
        state.speedCoefficient = this.speedCoefficient;

        io.updateState(state);
        Logger.getInstance().processInputs("Drive", state);
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public void setBrake() {
        io.setBrakeMode();
    }

    public void setCoast() {
        io.setCoastMode();
    }

    public void setWheelSpeeds(DifferentialDrive.WheelSpeeds speeds) {
        io.setWheelSpeeds(speeds);
    }

    public void slowDrive() {
        speedCoefficient = .33;
    }

    public void stop() {
        io.stop();
    }

    public void normalDrive() {
        speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;
    }

    public class TankDrive extends CommandBase {
        final Supplier<Double> leftJoystick;
        final Supplier<Double> rightJoystick;

        public TankDrive(Supplier<Double> leftJoystick, Supplier<Double> rightJoystick) {
            this.leftJoystick = leftJoystick;
            this.rightJoystick = rightJoystick;
        }

        @Override
        public void execute() {
            assert Drive.this.io != null;
            Drive.this.io.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(
                    adjustForDeadzone(leftJoystick.get()) * speedCoefficient,
                    adjustForDeadzone(rightJoystick.get()) * speedCoefficient
            ));
        }

        private double adjustForDeadzone(double in) {
            if (Math.abs(in) < Constants.JOYSTICK_DEADZONE) {
                return 0;
            }
            double sign = (in < 0) ? -Constants.MAX_DRIVETRAIN_SPEED : Constants.MAX_DRIVETRAIN_SPEED;
            return sign * Math.pow(in, 2);
        }
    }

    public class driveDistanceCommand extends CommandBase {
        private final double rightSpeed;
        private final double leftSpeed;
        private double[] initialPositions;
        private boolean hasReachedEnd = false;
        private final double encoderLimit;

        public driveDistanceCommand(double leftSpeed, double rightSpeed, double encoderLimit) {
            this.encoderLimit = encoderLimit;
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
            addRequirements(Drive.this);
        }

        @Override
        public void initialize() {
            initialPositions = Drive.this.state.encoderPositions;
            Drive.this.io.setWheelSpeeds(new DifferentialDrive.WheelSpeeds(leftSpeed, rightSpeed));
        }

        @Override
        public void execute() {
            double[] current_positions = Drive.this.state.encoderPositions;
            double leftDistance = (current_positions[0] - initialPositions[0]);
            double rightDistance = (current_positions[1] - initialPositions[1]);

            if (Math.abs(leftDistance + rightDistance) >= encoderLimit) {
                hasReachedEnd = true;
            }
        }

        @Override
        public boolean isFinished() {
            return hasReachedEnd;
        }

        @Override
        public void end(boolean isInterrupted){
            Drive.this.io.stop();
        }
    }
}
