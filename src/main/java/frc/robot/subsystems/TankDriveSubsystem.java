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

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public class TankDriveSubsystem extends SubsystemBase {
    private CANSparkMax primaryRightMotor;
    private CANSparkMax secondaryRightMotor;
    private CANSparkMax primaryLeftMotor;
    private CANSparkMax secondaryLeftMotor;
    private Encoder quadratureEncoder1;

    public TankDriveSubsystem() {
        primaryRightMotor = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);

        secondaryRightMotor = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
        secondaryRightMotor.follow(primaryRightMotor);

        primaryLeftMotor = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);

        secondaryLeftMotor = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
        secondaryLeftMotor.follow(primaryLeftMotor);

        quadratureEncoder1 = new Encoder(0, 1, false, EncodingType.k4X);
        quadratureEncoder1.setDistancePerPulse(1.0);
    }

    public TankDriveSubsystem(boolean invertRight, boolean invertLeft) { //optional inversion of motors
        primaryRightMotor = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
        primaryRightMotor.setInverted(invertRight);

        secondaryRightMotor = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
        secondaryRightMotor.setInverted(invertRight);
        secondaryRightMotor.follow(primaryRightMotor);

        primaryLeftMotor = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);

        secondaryLeftMotor = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);

        quadratureEncoder1 = new Encoder(0, 1, false, EncodingType.k4X);
        quadratureEncoder1.setDistancePerPulse(1.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public double getMotorSpeed(String LR) {
        if (LR.equals("LEFT")) {
            return primaryLeftMotor.get();
        }
        if (LR.equals("RIGHT")) {
            return primaryRightMotor.get();
        }
        return 0.0;
    }

    public void setMotor(double rightSpeed, double leftSpeed) {
        primaryLeftMotor.set(leftSpeed);
        primaryRightMotor.set(rightSpeed);
    }
    
    public class driveMotorCommand extends CommandBase {
        private DoubleSupplier m_rightSpeed;
        private DoubleSupplier m_leftSpeed;
    
        public driveMotorCommand(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed, TankDriveSubsystem subsystem) {
            m_rightSpeed = rightSpeed;
            m_leftSpeed = leftSpeed;
            addRequirements(subsystem);
        }
        
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            setMotor(m_rightSpeed.getAsDouble(), m_leftSpeed.getAsDouble());
        }
    }
}
