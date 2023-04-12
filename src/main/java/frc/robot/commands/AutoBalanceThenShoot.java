package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.Constants;
import frc.robot.utilities.NAVX;

public class AutoBalanceThenShoot extends CommandBase {
  private final NAVX gyro = NAVX.get();

  public AutoBalanceThenShoot() {
    addRequirements(TankDriveSubsystem.get());
  }

  @Override
  public void initialize() {
    System.out.println("Auto Balance w/ Shoot Called");
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
          ShooterSubsystem.get().StationToHybridShot(),
          IndexerSubsystem.get().new RunIndexer(),
          IntakeSubsystem.get().new RunIntake(),
          new WaitCommand(0.5)),
      new AutoBalance()
      ).schedule();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(gyro.getRoll()) <= 2.5;
  }
}