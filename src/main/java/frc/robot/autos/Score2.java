package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.utilities.TimedCommand;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class Score2 extends SequentialCommandGroup {
  public Score2() {
    addCommands(
        new ParallelRaceGroup(
          new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED),
          new WaitCommand(0.5)),
        Drive.get().new driveDistanceCommand(0.45, 0.45, 150),

        new TimedCommand(Intake.get().new IntakeUpDown(true), 0.5),
        new ParallelRaceGroup(
          new StartEndCommand(
                  Indexer.get()::run,
                  Indexer.get()::stop,
                  Indexer.get()),
          new StartEndCommand(
                  Intake.get()::run,
                  Intake.get()::stop,
                  Intake.get()
          ),
          Drive.get().new driveDistanceCommand(0.3, 0.3, 20)),
        
        Drive.get().new driveDistanceCommand(-0.45, -0.45, 180),
        new InstantCommand(Drive.get()::setBrake),
        new WaitCommand(0.5),
        new TimedCommand(new ShootCube(Constants.L1_SHOOTER_SPEED), 0.5),
        Drive.get().new driveDistanceCommand(0.3, 0.3, 150),
        new InstantCommand(Intake.get()::setIntakePositionCoast)
    );
  }
}