package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceThenShoot;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class TwoPieceBalanceRight extends SequentialCommandGroup {

  public TwoPieceBalanceRight() {
    addCommands(
      new ParallelRaceGroup(ShooterSubsystem.get().AutoL2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 150),
      new ParallelRaceGroup(TankDriveSubsystem.get().new driveDistanceCommand(0.43, 0.3, 65), new LoadCube()),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.43, -0.3, 55),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 80),
      new InstantCommand(() -> TankDriveSubsystem.get().setBrake()),
      // new ParallelRaceGroup(ShooterSubsystem.get().CommunityShot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(.5)),
      new AutoBalanceThenShoot()
    );
  }
}