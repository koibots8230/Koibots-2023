// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
  /** Creates a new TimedCommand. */
  Command m_Command;
  int m_repitions;

  public TimedCommand(Command command, int repititions) {
    this.m_Command = command;
    this.m_repitions = repititions;
  }

  public TimedCommand(Command command, double seconds) {
    this(command, (int) (seconds * 50));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Command.schedule();
  }

  @Override
  public void execute() {
    m_repitions--;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_repitions <= 0;
  }

  @Override
  public void end(boolean interrupted) {
      m_Command.cancel();
  }
}
