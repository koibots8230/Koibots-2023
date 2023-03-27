// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PauseCommand extends CommandBase {
  /** Creates a new waitCommand. */
  int m_waitPeriod;
  int count = 0;
  public PauseCommand(int waitPeriod) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_waitPeriod = waitPeriod;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new AutoBalanceCommand().schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (count >= m_waitPeriod) {
      return true;
    }
    return false;
  }
}