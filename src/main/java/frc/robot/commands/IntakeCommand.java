package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final Boolean m_fwd;

    public IntakeCommand(IntakeSubsystem subsystem, Boolean fwd) {
        m_intake = subsystem;
        m_fwd = fwd;
        addRequirements(m_intake);
    }
    
    @Override
    public void initialize() {

        
        m_intake.turnOn(m_fwd);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.turnOff();
    }
}
