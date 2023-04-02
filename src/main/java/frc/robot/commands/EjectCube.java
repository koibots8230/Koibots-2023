package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectCube extends ParallelCommandGroup {

  public EjectCube() {
    addCommands(
      IntakeSubsystem.get().new RunIntakeReverse(),
      IndexerSubsystem.get().new RunIndexerReverse()
    );
  }
}
