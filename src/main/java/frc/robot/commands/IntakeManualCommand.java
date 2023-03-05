package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeManualCommand extends CommandBase {
    IntakeSubsystem intake;
    boolean up;
    boolean end = false;

    public IntakeManualCommand(IntakeSubsystem _intake, boolean _up) {
        addRequirements(_intake);
        intake = _intake;
        up = _up;
    }

    public void initialize() {
        if (up) {
            intake.setRaiseIntakeSpeed(Constants.RAISE_SPEED);
        } else {
            intake.setRaiseIntakeSpeed(-Constants.RAISE_SPEED);
        }
    }

    public void periodic() {
        if (intake.getRaiseMotorCurrent() >= 70) {
            end = true;
        }
    }

    public boolean isFinished() {
        return end;
    }

    public void end(boolean interrupted) {
        intake.setRaiseIntakeSpeed(0);
    }
}
