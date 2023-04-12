package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.*;

public class CommunityBalance extends SequentialCommandGroup {

  public CommunityBalance() {
    addCommands(
      new ParallelRaceGroup(
      ShooterSubsystem.get().AutoL2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 170),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 85),
      new InstantCommand(() -> TankDriveSubsystem.get().setBrake()),
      new AutoBalance()
    );
  }
}