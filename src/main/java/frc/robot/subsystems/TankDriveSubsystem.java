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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class TankDriveSubsystem extends SubsystemBase {
    private CANSparkMax primaryRightMotor;
    private CANSparkMax secondaryRightMotor;
    private CANSparkMax primaryLeftMotor;
    private CANSparkMax secondaryLeftMotor;
    private Encoder quadratureEncoder1;

    public TankDriveSubsystem() {
        final CANSparkMax primaryRightMotor = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
        addChild("PrimaryRightMotor",(Sendable) primaryRightMotor);

        final CANSparkMax secondaryRightMotor = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
        addChild("SecondaryRightMotor", (Sendable) secondaryRightMotor);
        secondaryRightMotor.follow(primaryRightMotor);

        final CANSparkMax primaryLeftMotor = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
        addChild("PrimaryRightMotor", (Sendable) primaryLeftMotor);

        final CANSparkMax secondaryLeftMotor = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
        addChild("SecondaryLeftMotor",(Sendable) secondaryLeftMotor);
        secondaryLeftMotor.follow(primaryLeftMotor);

        quadratureEncoder1 = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("Quadrature Encoder 1",quadratureEncoder1);
        quadratureEncoder1.setDistancePerPulse(1.0);
    }

    public TankDriveSubsystem(boolean invertRight) { //optional inversion of motors
        final CANSparkMax primaryRightMotor = new CANSparkMax(Constants.kRightMotor1Port, MotorType.kBrushless);
        addChild("PrimaryRightMotor",(Sendable) primaryRightMotor);
        primaryRightMotor.setInverted(invertRight);

        final CANSparkMax secondaryRightMotor = new CANSparkMax(Constants.kRightMotor2Port, MotorType.kBrushless);
        addChild("SecondaryRightMotor", (Sendable) secondaryRightMotor);
        secondaryRightMotor.setInverted(invertRight);

        final CANSparkMax primaryLeftMotor = new CANSparkMax(Constants.kLeftMotor1Port, MotorType.kBrushless);
        addChild("PrimaryRightMotor", (Sendable) primaryLeftMotor);

        final CANSparkMax secondaryLeftMotor = new CANSparkMax(Constants.kLeftMotor2Port, MotorType.kBrushless);
        addChild("SecondaryLeftMotor",(Sendable) secondaryLeftMotor);

        quadratureEncoder1 = new Encoder(0, 1, false, EncodingType.k4X);
        addChild("Quadrature Encoder 1",quadratureEncoder1);
        quadratureEncoder1.setDistancePerPulse(1.0);
    }

    // The left-side drive encoder
    private final Encoder m_leftEncoder =
    new Encoder(
      Constants.kLeftMotor1Port,
      Constants.kLeftMotor2Port);
    // The right-side drive encoder
    private final Encoder m_rightEncoder =
    new Encoder(
      Constants.kRightMotor1Port,
      Constants.kRightMotor2Port);
    
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
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
        if (LR == "LEFT") {
            return this.primaryLeftMotor.get();
        }
        if (LR == "RIGHT") {
            return this.primaryRightMotor.get();
        }
        return 0.0;
    }

    public void setMotor(String LR, double speed) {
        if (LR == "LEFT") {
            this.primaryLeftMotor.set(speed);
            return;
        }
        if (LR == "RIGHT") {
            this.primaryRightMotor.set(speed);
            return;
        }
        return;
    }

    
}
