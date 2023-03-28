package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PauseCommand extends CommandBase {
  int m_waitPeriod;
  int count = 0;
  public PauseCommand(int waitPeriod) {
    m_waitPeriod = waitPeriod;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    count++;
  }

  @Override
  public void end(boolean interrupted) {
    new AutoBalanceCommand().schedule();
  }

  @Override
  public boolean isFinished() {
    if (count >= m_waitPeriod) {
      return true;
    }
    return false;
  }
}