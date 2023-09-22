package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadCube extends ParallelRaceGroup {

  public LoadCube() {
    addCommands(
      IntakeSubsystem.get().new RunIntake(),
      IndexerSubsystem.get().new RunIndexer()
    );
  }
}
