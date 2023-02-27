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

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;

    private final CANSparkMax firstConveyer;
    private final RelativeEncoder conveyerEncoder;
    private final CANSparkMax leftStarWheelsMotor;
    private final CANSparkMax rightStarWheelsMotor;

    // This motor raises and lowers the intake:
    private final CANSparkMax raiseIntakeMotor;
    private final RelativeEncoder raiseIntakeEncoder;

    private double intakePosition; // This variable refers to the incline of the intake IN DEGREES

    private final AnalogInput topHallEffectSensor;
    private final AnalogInput bottomHallEffectSensor;
    private IntakeState state;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeEncoder = intakeMotor.getEncoder();

        firstConveyer = new CANSparkMax(Constants.MIDTAKE_MOTOR, MotorType.kBrushless); 
        firstConveyer.setInverted(false);
        conveyerEncoder = firstConveyer.getEncoder();

        rightStarWheelsMotor = new CANSparkMax(Constants.STAR_WHEELS_MOTOR_L, MotorType.kBrushless);
        rightStarWheelsMotor.setInverted(true);
        leftStarWheelsMotor = new CANSparkMax(Constants.STAR_WHEELS_MOTOR_R, MotorType.kBrushless);
        leftStarWheelsMotor.setInverted(true);
        leftStarWheelsMotor.follow(rightStarWheelsMotor);

        // raiseIntakeMotor:
        raiseIntakeMotor = new CANSparkMax(Constants.RAISE_INTAKE_MOTOR, MotorType.kBrushless);
        raiseIntakeMotor.setInverted(false);
        raiseIntakeEncoder = raiseIntakeMotor.getEncoder();
        raiseIntakeEncoder.setPosition(0);
        intakePosition = 0;

        // Hall effect sensors
        topHallEffectSensor = new AnalogInput(0); // Change port number when testing the code
        bottomHallEffectSensor = new AnalogInput(1); // Change port numer when testing the code

        state = IntakeState.TOP;
    }

    enum IntakeState {
        BOTTOM,
        TOP,
        MOVE,
        CALIBRATE
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double inVelocity = intakeEncoder.getVelocity();
        double InCurrent = intakeMotor.getOutputCurrent(); 
        SmartDashboard.putNumber("Intake Motor Speed (RPM)", inVelocity);
        SmartDashboard.putNumber("Main Battery Current (A)", InCurrent);
        double midVelocity = conveyerEncoder.getVelocity();
        double midCurrent = firstConveyer.getOutputCurrent();
        SmartDashboard.putNumber("Midtake Motor Current (A)", midCurrent);
        SmartDashboard.putNumber("Midtake Motor Speed (RPM)", midVelocity);
        state = getIntakeState();

    }

    @Override
    public void simulationPeriodic() {
    }

    public void turnOn() {
        intakeMotor.set(Constants.RUNNING_SPEED);
        firstConveyer.set(Constants.RUNNING_SPEED);
        rightStarWheelsMotor.set(Constants.RUNNING_SPEED);
    }

    public void turnOn(Boolean Forwards) {
        if (Forwards){
            intakeMotor.set(-Constants.RUNNING_SPEED);
            firstConveyer.set(Constants.RUNNING_SPEED);
            rightStarWheelsMotor.set(Constants.RUNNING_SPEED);
        } else {
            intakeMotor.set(Constants.RUNNING_SPEED);
            firstConveyer.set(-Constants.RUNNING_SPEED);
            rightStarWheelsMotor.set(-Constants.RUNNING_SPEED);
        }
    }

    public void turnOff() {
        intakeMotor.set(0);
        firstConveyer.set(0);
        rightStarWheelsMotor.set(0);
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

    public IntakeState getIntakeState() {
        // Returns the correct hall effect sensor based off current intake state:
        if (topHallEffectSensor.getVoltage() > Constants.HALL_EFFECT_SENSOR_TRIGGERED) {
            return IntakeState.BOTTOM;
            // If the top hall effect sensor is triggered, it means that the intake is top.
            // Assuming that we want to go down, the function returns the BOTTOM sensor.
        }
        if (bottomHallEffectSensor.getVoltage() > Constants.HALL_EFFECT_SENSOR_TRIGGERED) {
            return IntakeState.TOP;
            // The OPPOSITE goes for the top sensor.
        } 
        //if none of these conditions are satisified, we know that we are not in intake state top or bototm
        return IntakeState.MOVE;
    }

    public class FlipIntake extends CommandBase {
        IntakeSubsystem m_intake;
        boolean end = false;
        AnalogInput hallEffectSensor;

        public FlipIntake(IntakeSubsystem subsystem) {
            m_intake = subsystem;
            addRequirements(m_intake);
        }

        @Override
        public void initialize() {
            IntakeState top;
            System.out.println("Intake is moving");
            top = m_intake.getIntakeState();
            //use get just incase intake isnt run
            switch(top){
                case TOP:
                    m_intake.setRaiseIntakeSpeed(-Constants.RAISE_SPEED);
                case BOTTOM:
                    m_intake.setRaiseIntakeSpeed(Constants.RAISE_SPEED);
                case MOVE:
                    //dont use this command if we're in a move state, so just end
                    //leaving this in case we need it in future challenges
                    end = true;
                case CALIBRATE:
                    //move down to calibrate if we dont know our position
                    m_intake.setRaiseIntakeSpeed(-Constants.RAISE_SPEED);
            }
        }

        @Override
        public void execute() {
            if (Math.abs(m_intake.getRaiseMotorCurrent()) > Constants.CURRENT_ZONE_AMPS || hallEffectSensor.getVoltage() > Constants.HALL_EFFECT_SENSOR_TRIGGERED) {
                if (m_intake.getRaiseEncoder().getPosition() > Constants.INTAKE_UP_POSITION || m_intake.getRaiseEncoder().getPosition() < Constants.INTAKE_DOWN_POSITION) {
                m_intake.setRaiseIntakeSpeed(0);
                end = true;
                } 
            } 
        }

        @Override
        public boolean isFinished() {
            return end;
        }

    }
}