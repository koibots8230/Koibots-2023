package frc.robot.autos;

import frc.robot.commands.LoadCube;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class Score2 extends SequentialCommandGroup {
  public Score2() {
    addCommands(
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          ShooterSubsystem.get().L2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
        IntakePositionSubsystem.get().new FlipIntake()
        ),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 130),
      new LoadCube(),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 130),
      ShooterSubsystem.get().L1Shot(),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 100),
      new InstantCommand(() -> IntakePositionSubsystem.get().setCoast())
    );
  }
}