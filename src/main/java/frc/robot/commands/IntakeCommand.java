package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_is_forwards;
    private final BooleanSupplier m_is_backwards;

    public IntakeCommand(IntakeSubsystem subsystem, BooleanSupplier is_forwards, BooleanSupplier is_backwards) {
        m_intake = subsystem;
        m_is_forwards = is_forwards;
        m_is_backwards = is_backwards;
        addRequirements(m_intake);
    }
    
    @Override
    public void execute() {
        if (m_is_forwards.getAsBoolean() ^ m_is_backwards.getAsBoolean()){
            m_intake.turnOn(m_is_forwards.getAsBoolean());
        } 
        else {
            // Turn off the intake
            m_intake.turnOff();
        }
    }
}
