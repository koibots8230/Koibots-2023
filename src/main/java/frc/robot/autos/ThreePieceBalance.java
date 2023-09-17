package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.commands.AutoBalanceThenShoot;
import frc.robot.commands.LoadCube;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;


public class ThreePieceBalance extends SequentialCommandGroup {
  final double mainSpeed = 0.35;
  final double curveSpeed = 0.48;
  public ThreePieceBalance() {
    addCommands(
      new ParallelRaceGroup(new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED), new WaitCommand(0.5)),
      Drive.get().new driveDistanceCommand(mainSpeed, mainSpeed, 150),

      new ParallelRaceGroup(
        new ParallelCommandGroup(Drive.get().new driveDistanceCommand(mainSpeed, curveSpeed, 65), Intake.get().new IntakeUpDown(true)),
        new LoadCube()),
      Drive.get().new driveDistanceCommand(-mainSpeed, -curveSpeed, 55),

      Drive.get().new driveDistanceCommand(-mainSpeed, -mainSpeed, 30),
      new ParallelRaceGroup(
        Drive.get().new driveDistanceCommand(-mainSpeed, -mainSpeed, 20),
        new ShootCube(Constants.COMMUNITY_SHOOTER_SPEED),
        new StartEndCommand(
                Intake.get()::run,
                Intake.get()::stop,
                Intake.get())),
      Drive.get().new driveDistanceCommand(mainSpeed, mainSpeed, 40),

      new ParallelRaceGroup(Drive.get().new driveDistanceCommand(curveSpeed, mainSpeed, 65), new LoadCube()),
      Drive.get().new driveDistanceCommand(-curveSpeed, -mainSpeed, 55),

      Drive.get().new driveDistanceCommand(-mainSpeed, -mainSpeed, 80),
      new InstantCommand(() -> Drive.get().setBrake()),
      // new ParallelRaceGroup(ShooterSubsystem.get().CommunityShot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(.5)),
      new AutoBalanceThenShoot()
    );
  }
}