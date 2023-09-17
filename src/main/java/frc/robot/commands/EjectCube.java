package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class EjectCube extends ParallelCommandGroup {

  public EjectCube() {
    addCommands(
      new StartEndCommand(
              Intake.get()::runReversed,
              Intake.get()::stop,
              Intake.get()
      ),
      new StartEndCommand(
              Indexer.get()::runReversed,
              Indexer.get()::stop,
              Indexer.get()
      )
    );
  }
}
