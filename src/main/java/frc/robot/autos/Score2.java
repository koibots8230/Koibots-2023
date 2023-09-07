package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Utilities.TimedCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class Score2 extends SequentialCommandGroup {
  public Score2() {
    addCommands(
        new ParallelRaceGroup(
          new Shoot(Constants.AUTO_L2_SHOOTER_SPEED),
          new WaitCommand(0.5)),
        TankDriveSubsystem.get().new driveDistanceCommand(0.45, 0.45, 150),

        new TimedCommand(IntakePositionSubsystem.get().new IntakeUpDown(true), 0.5),
        new ParallelRaceGroup(
          IndexerSubsystem.get().new RunIndexer(),
          IntakeSubsystem.get().new RunIntake(),
          TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 20)),
        
        TankDriveSubsystem.get().new driveDistanceCommand(-0.45, -0.45, 180),
        new InstantCommand(TankDriveSubsystem.get()::setBrake),
        new WaitCommand(0.5),
        new TimedCommand(new Shoot(Constants.L1_SHOOTER_SPEED), 0.5),
        TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 150),
        new InstantCommand(IntakePositionSubsystem.get()::setCoast)
    );
  }
}