// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class TankDriveSubsystem extends SubsystemBase {
    private static CANSparkMax primaryRightMotor;
    private CANSparkMax secondaryRightMotor;
    private static CANSparkMax primaryLeftMotor;
    private CANSparkMax secondaryLeftMotor;
    private double speedCoefficient = 1;
    

    public TankDriveSubsystem() {
        primaryRightMotor = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);

        secondaryRightMotor = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
        secondaryRightMotor.follow(primaryRightMotor);

        primaryLeftMotor = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);

        secondaryLeftMotor = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
        secondaryLeftMotor.follow(primaryLeftMotor);

        
    }

    public TankDriveSubsystem(boolean invertRight, boolean invertLeft) { // optional inversion of motors
        this();

        primaryRightMotor.setInverted(invertRight);
        primaryLeftMotor.setInverted(invertLeft);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public SparkMaxPIDController getLeftPID() {
        return primaryLeftMotor.getPIDController();
    }

    public SparkMaxPIDController getRightPID() {
        return primaryRightMotor.getPIDController();
    }

    public static void setMotor(double leftSpeed, double rightSpeed) {
        primaryLeftMotor.set(leftSpeed);
        primaryRightMotor.set(rightSpeed);
    }
    
    public void setSpeed(Boolean Increase){
        if (Increase) {
            //Only need increase - if it's called and Increase is false than decrease is pressed instead
            if (speedCoefficient < 1) {
                speedCoefficient += 0.05;
            }
        } else {
            if (speedCoefficient > 0){
                speedCoefficient -= 0.05;
            }
        }
    }

    public class driveMotorCommand extends CommandBase {
        private DoubleSupplier m_rightSpeed;
        private DoubleSupplier m_leftSpeed;
        private SparkMaxPIDController m_rightPID;
        private SparkMaxPIDController m_leftPID;
        private TankDriveSubsystem m_DriveSubsystem;

        public driveMotorCommand(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed, TankDriveSubsystem subsystem) {
            m_rightSpeed = rightSpeed;
            m_leftSpeed = leftSpeed;
            m_DriveSubsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            m_rightPID = m_DriveSubsystem.getRightPID();
            m_leftPID = m_DriveSubsystem.getLeftPID();

            m_rightPID.setOutputRange(-1, 1);
            m_leftPID.setOutputRange(-1, 1);

            m_rightPID.setP(Constants.kp);
            m_leftPID.setP(Constants.kp);

            m_rightPID.setI(Constants.ki);
            m_leftPID.setI(Constants.ki);

            m_rightPID.setD(Constants.kd);
            m_leftPID.setD(Constants.kd);
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            m_leftPID.setReference(adjustForDeadzone(m_leftSpeed.getAsDouble()), CANSparkMax.ControlType.kDutyCycle);
            m_rightPID.setReference(adjustForDeadzone(m_rightSpeed.getAsDouble()), CANSparkMax.ControlType.kDutyCycle);
        }

        private double adjustForDeadzone(double in) {
            if (Math.abs(in) < Constants.DEADZONE) {
                return 0;
            }
            double sign = (int) Math.signum(in);
            double out = Math.abs(sign);
            out *= (1 / 1 - Constants.DEADZONE);
            out *= sign * out;
            return out;
        }
    }

}