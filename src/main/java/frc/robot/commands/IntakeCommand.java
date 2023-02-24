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
    public void initialize() {
        m_intake.turnOn(m_intake.getForward());
    }
    @Override
    public boolean isFinished(){
        return m_runOrNot.getAsBoolean();
    }
    @Override
    public void end(boolean interrupted) {
        m_intake.turnOff();
    }
}
