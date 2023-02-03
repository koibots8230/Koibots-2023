package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_is_on;

    public IntakeCommand(IntakeSubsystem subsystem, BooleanSupplier is_on) {
        m_intake = subsystem;
        m_is_on = is_on;
        addRequirements(m_intake);
    }
    
    @Override
    public void execute() {
        if (m_is_on.getAsBoolean()) {
            // Turn on the intake
            m_intake.turnOn();
        }
        else {
            // Turn off the intake
            m_intake.turnOff();
        }
    }
}
