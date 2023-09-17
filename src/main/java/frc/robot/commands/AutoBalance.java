package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.utilities.NAVX;
import frc.robot.utilities.PauseCommand;
import frc.robot.subsystems.drive.Drive;

public class AutoBalance extends CommandBase {
  private final NAVX gyro;

  public AutoBalance() {
    gyro = NAVX.get();
    addRequirements(Drive.get());
  }

  @Override
  public void execute() {
    Drive.get().setWheelSpeeds(
            new DifferentialDrive.WheelSpeeds(
                    Constants.AUTO_SPEED * Math.signum(gyro.getRoll()),
                    Constants.AUTO_SPEED * Math.signum(gyro.getRoll())
            )
    );
  }

  @Override
  public void end(boolean interrupted) {
    Drive.get().stop();
    new PauseCommand(new AutoBalance(), 25).schedule();
  }

  @Override
  public boolean isFinished() {
      return Math.abs(gyro.getRoll()) <= 2.5;
  }
}