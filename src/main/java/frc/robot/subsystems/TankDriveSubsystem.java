package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.NAVX;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class TankDriveSubsystem extends SubsystemBase{
    private static final TankDriveSubsystem tankDriveSubsystem = new TankDriveSubsystem();

    private final CANSparkMax primaryRightMotor;
    private final CANSparkMax secondaryRightMotor;
    private final CANSparkMax primaryLeftMotor;
    private final CANSparkMax secondaryLeftMotor;

    private final RelativeEncoder leftPrimaryRelativeEncoder;
    private final RelativeEncoder rightPrimaryRelativeEncoder;

    private double previousLeftEncoder;
    private double previousRightEncoder;

    private final DifferentialDriveOdometry odometry;

    private double speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;

    TankDriveSubsystem() {

        primaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
        primaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);

        primaryLeftMotor.setInverted(true);
        primaryRightMotor.setInverted(false);

        secondaryRightMotor.follow(primaryRightMotor);
        secondaryLeftMotor.follow(primaryLeftMotor);

        leftPrimaryRelativeEncoder = primaryLeftMotor.getEncoder();
        rightPrimaryRelativeEncoder = primaryRightMotor.getEncoder();

        leftPrimaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        rightPrimaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);

        odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(NAVX.get().getAngle())),
                leftPrimaryRelativeEncoder.getPosition(), rightPrimaryRelativeEncoder.getPosition());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Side velocity", leftPrimaryRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("Right Side velocity", rightPrimaryRelativeEncoder.getVelocity());
        
        if (Math.abs(NAVX.get().getRoll()) > .75) {
            odometry.update(
                NAVX.get().getRotation2d(),
                leftPrimaryRelativeEncoder.getPosition() - previousLeftEncoder * Math.cos(Math.toRadians(NAVX.get().getRoll())) + previousLeftEncoder,
                rightPrimaryRelativeEncoder.getPosition() - previousRightEncoder * Math.cos(Math.toRadians(NAVX.get().getRoll())) + previousRightEncoder);
        } else {
            odometry.update(
                NAVX.get().getRotation2d(),
                leftPrimaryRelativeEncoder.getPosition(),
                rightPrimaryRelativeEncoder.getPosition()); 
            previousLeftEncoder = leftPrimaryRelativeEncoder.getPosition();
            previousRightEncoder = rightPrimaryRelativeEncoder.getPosition();
        }
    }

    // ================================Getters================================ \\

    public double[] getVoltages() {
        return new double[] { primaryLeftMotor.getAppliedOutput(), primaryRightMotor.getAppliedOutput()};
    }

    public static TankDriveSubsystem get() {
        return tankDriveSubsystem;
    }


    public double[] getEncoderPositions() {
        return (new double[] { leftPrimaryRelativeEncoder.getPosition() / Constants.DRIVE_ROTATIONS_TO_DISTANCE, rightPrimaryRelativeEncoder.getPosition() / Constants.DRIVE_ROTATIONS_TO_DISTANCE });
    }

    // ================================Setters================================ \\

    public void resetEncoders() {
        leftPrimaryRelativeEncoder.setPosition(0);
        rightPrimaryRelativeEncoder.setPosition(0);
    }

    public void setMotor(double leftSpeed, double rightSpeed) {
        primaryLeftMotor.set(leftSpeed);
        primaryRightMotor.set(rightSpeed);
    }

    public void startSlowMode() {
        speedCoefficient = .33;
    }

    public void stopSlowMode() {
        speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;
    }

    public void setBrake() {
        primaryLeftMotor.setIdleMode(IdleMode.kBrake);
        primaryRightMotor.setIdleMode(IdleMode.kBrake);
        secondaryLeftMotor.setIdleMode(IdleMode.kBrake);
        secondaryRightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoast() {
        primaryLeftMotor.setIdleMode(IdleMode.kCoast);
        primaryRightMotor.setIdleMode(IdleMode.kCoast);
        secondaryLeftMotor.setIdleMode(IdleMode.kCoast);
        secondaryRightMotor.setIdleMode(IdleMode.kCoast);
    }

    // ================================Commands================================ \\

    public class driveMotorCommand extends CommandBase {
        private final DoubleSupplier rightSpeed;
        private final DoubleSupplier leftSpeed;

        public driveMotorCommand(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
            this.rightSpeed = rightSpeed;
            this.leftSpeed = leftSpeed;
            addRequirements(TankDriveSubsystem.this);
        }

        @Override
        public void execute() {
            TankDriveSubsystem.this.setMotor(
                    adjustForDeadzone(leftSpeed.getAsDouble()) * speedCoefficient,
                    adjustForDeadzone(rightSpeed.getAsDouble()) * speedCoefficient);
        }

        private double adjustForDeadzone(double in) {
            if (Math.abs(in) < Constants.THUMBSTICK_DEADZONE) {
                return 0;
            }
            double sign = (in < 0) ? -Constants.MAX_DRIVETRAIN_SPEED : Constants.MAX_DRIVETRAIN_SPEED;
            return sign * Math.pow(in, 2);
        }
    }

    public class driveDistanceCommand extends CommandBase {
        private final double rightSpeed;
        private final double leftSpeed;
        private final double encoderLimit;
        private double[] initialPositions;
        private boolean hasReachedEnd = false;


        public driveDistanceCommand(double leftSpeed, double rightSpeed, double encoder_limit) {
            encoderLimit = encoder_limit;
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
            addRequirements(TankDriveSubsystem.this);
        }
        
        @Override
        public void initialize() {
            initialPositions = TankDriveSubsystem.this.getEncoderPositions();
            TankDriveSubsystem.this.setMotor(leftSpeed, rightSpeed);
        }

        @Override
        public void execute() {
            double[] current_positions = TankDriveSubsystem.this.getEncoderPositions();
            double l_dif = (current_positions[0] - initialPositions[0]);
            double r_dif = (current_positions[1] - initialPositions[1]);

            if (Math.abs(l_dif + r_dif) >= encoderLimit) {
                hasReachedEnd = true;
            }
        }
        
        @Override 
        public boolean isFinished() {
            return hasReachedEnd;
        }

        @Override 
        public void end(boolean isInterrupted){
            TankDriveSubsystem.this.setMotor(0, 0);
        }
    }
}