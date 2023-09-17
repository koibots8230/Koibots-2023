package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.utilities.TimedCommand;
import frc.robot.commands.ShootCube;
import frc.robot.commands.AutoBalanceThenShoot;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.drive.Drive;

public class CommunityPickupBalance extends SequentialCommandGroup {
  public CommunityPickupBalance() {
    addCommands(
      new TimedCommand(new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED), 0.5),
      new ParallelRaceGroup(Drive.get().new driveDistanceCommand(0.3, 0.435, 65), new LoadCube()),
      Drive.get().new driveDistanceCommand(-0.3, -0.435, 55),
      Drive.get().new driveDistanceCommand(-0.3, -0.3, 80),
      new InstantCommand(() -> Drive.get().setBrake()),
      // new ParallelRaceGroup(ShooterSubsystem.get().CommunityShot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(.5)),
      new AutoBalanceThenShoot()
    );
  }
}