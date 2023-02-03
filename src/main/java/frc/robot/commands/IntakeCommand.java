package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_on;

    public IntakeCommand(IntakeSubsystem subsystem, BooleanSupplier on) {
        m_intake = subsystem;
        m_on = on;
        addRequirements(m_intake);
    }
    
    @Override
    public void execute() {
        if (m_on.getAsBoolean()) {
            // Turn on the intake
        }
        else {
            // Turn off the intake
        }
    }
}
