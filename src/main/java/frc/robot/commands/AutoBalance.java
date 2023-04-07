package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.Constants;
import frc.robot.Utilities.NAVX;
import frc.robot.Utilities.PauseCommand;

public class AutoBalance extends CommandBase {
  private NAVX gyro;

  public AutoBalance() {
    gyro = NAVX.get();
    addRequirements(TankDriveSubsystem.get());
  }

  @Override
  public void initialize() {
    System.out.println("Auto Balance Called");
  }

  @Override
  public void execute() {

    TankDriveSubsystem.get().setMotor(  
      Constants.AUTO_SPEED * Math.signum(gyro.getRoll()),
      Constants.AUTO_SPEED * Math.signum(gyro.getRoll()));
    
  }

  @Override
  public void end(boolean interrupted) {
    TankDriveSubsystem.get().setMotor(0, 0);
    new PauseCommand(new AutoBalance(), 25).schedule();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(gyro.getRoll()) <= 2.5) {
      return true;
    }
    return false;
  }
}