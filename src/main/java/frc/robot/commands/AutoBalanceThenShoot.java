package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.utilities.NAVX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;

public class AutoBalanceThenShoot extends CommandBase {
  private final NAVX gyro;

  public AutoBalanceThenShoot() {
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
    new SequentialCommandGroup(
      new WaitCommand(.5),
      new ParallelRaceGroup(
          new ShootCube(Constants.STATION_TO_HYBRID_SHOOTER_SPEED),
          new StartEndCommand(
                  Intake.get()::run,
                  Intake.get()::stop,
                  Intake.get()),
          new WaitCommand(0.5)),
      new AutoBalance()
      ).schedule();
  }

  @Override
  public boolean isFinished() {
      return Math.abs(gyro.getRoll()) <= 2.5;
  }
}