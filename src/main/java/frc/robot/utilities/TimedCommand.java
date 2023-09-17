// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
  /** Creates a new TimedCommand. */
  final Command command;
  int repetitions;

  public TimedCommand(Command command, int repetitions) {
    this.command = command;
    this.repetitions = repetitions;
  }

  public TimedCommand(Command command, double seconds) {
    this(command, (int) (seconds * 50));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.schedule();
  }

  @Override
  public void execute() {
    repetitions--;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return repetitions <= 0;
  }

  @Override
  public void end(boolean interrupted) {
      command.cancel();
  }
}
