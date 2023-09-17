package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class LoadCube extends ParallelRaceGroup {

  public LoadCube() {
    addCommands(
      new StartEndCommand(
              Intake.get()::run,
              Intake.get()::stop,
              Intake.get()
      ),
      Indexer.get().new RunUntilBeam()
    );
  }
}
