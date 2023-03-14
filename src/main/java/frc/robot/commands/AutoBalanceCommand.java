// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.Constants;
import frc.robot.Utilities.NAVX;
import frc.robot.commands.AutoBalanceCommand;

public class AutoBalanceCommand extends CommandBase {
    private final NAVX gyro;
    private double Factor = 1;

    /** Creates a new AutoBalanceCommand. */
    public AutoBalanceCommand() {
      // Use addRequirements() here to declare subsystem dependencies.
      gyro = NAVX.get();
      addRequirements(TankDriveSubsystem.get());
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightDirection;
    double leftDirection;
    if (Math.abs(gyro.getWorldLinearAccelX()) > 0.8) {
      rightDirection = gyro.getRoll() * gyro.getWorldLinearAccelY();
      leftDirection = gyro.getRoll() * gyro.getWorldLinearAccelY() * -1;
    } else {
      rightDirection = gyro.getRoll();
      leftDirection = gyro.getRoll();
    }
    
    TankDriveSubsystem.get().setMotor(-Constants.AUTO_SPEED * Factor * Math.signum(rightDirection), -Constants.AUTO_SPEED * Factor * Math.signum(leftDirection));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TankDriveSubsystem.get().setMotor(0, 0);
    if (!interrupted) {
      new waitCommand(25);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(gyro.getRoll()) <= 2.5) {
      Factor -= 0.05;
      return true;
    }
    return false;
  }
}