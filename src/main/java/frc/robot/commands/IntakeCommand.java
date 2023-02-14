package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final BooleanSupplier m_runOrNot;

    public IntakeCommand(IntakeSubsystem subsystem, BooleanSupplier run) {
        m_intake = subsystem;
        m_runOrNot = run;
        addRequirements(m_intake);
    }
    
    @Override
    public void execute() {
        if (m_runOrNot.getAsBoolean()) {
            m_intake.turnOn(m_intake.getForward());
        } else {
            m_intake.turnOff();
        }
    }
}
