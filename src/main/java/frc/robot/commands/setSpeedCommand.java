// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;

public class setSpeedCommand extends CommandBase {
  private Boolean m_speedIncrease;
  private TankDriveSubsystem m_DriveSubsystem;

  public setSpeedCommand(Boolean Up, TankDriveSubsystem subsystem){
      m_speedIncrease = Up;
      m_DriveSubsystem = subsystem;
      addRequirements(m_DriveSubsystem);
  }
  @Override 
  public void execute() {
      m_DriveSubsystem.setSpeed(m_speedIncrease);
  }
  @Override
  public boolean isFinished(){
      return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
