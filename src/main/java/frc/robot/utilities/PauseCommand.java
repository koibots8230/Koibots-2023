package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PauseCommand extends CommandBase {
    final Command command;
    int iterations;

    public PauseCommand(Command command, int iterations) {
        this.command = command;
        this.iterations = iterations;
    }

    @Override
    public void execute() {
        iterations--;
    }

    @Override
    public void end (boolean interrupted) {
        this.command.schedule();
    }

    @Override
    public boolean isFinished() {
        return iterations <= 0;
    }
}
