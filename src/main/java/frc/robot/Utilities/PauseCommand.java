package frc.robot.Utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PauseCommand extends CommandBase {
    Command commandToRun;
    int iterations;

    public PauseCommand(Command command, int count) {
        commandToRun = command;
        iterations = count;
    }

    @Override
    public void execute() {
        iterations--;
    }

    @Override
    public void end (boolean interrupted) {
        commandToRun.schedule();
    }

    @Override
    public boolean isFinished() {
        return iterations <= 0;
    }
}
