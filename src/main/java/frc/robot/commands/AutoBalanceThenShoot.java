package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.Constants;
import frc.robot.Utilities.NAVX;

public class AutoBalanceThenShoot extends CommandBase {
  private NAVX gyro;

  public AutoBalanceThenShoot() {
    gyro = NAVX.get();
    addRequirements(TankDriveSubsystem.get());
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
    new SequentialCommandGroup(
      new WaitCommand(.5),
      new ParallelRaceGroup(
          new ShootCube(Constants.STATION_TO_HYBRID_SHOOTER_SPEED),
          IntakeSubsystem.get().new RunIntake(),
          new WaitCommand(0.5)),
      new AutoBalance()
      ).schedule();
  }

  @Override
  public boolean isFinished() {
    if (Math.abs(gyro.getRoll()) <= 2.5) {
      return true;
    }
    return false;
  }
}