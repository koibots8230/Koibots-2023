// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.commands.LoadCube;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score2 extends SequentialCommandGroup {
  /** Creates a new shootMove. */
  public Score2() {
    addCommands(
      new ParallelCommandGroup(
        new ParallelRaceGroup(
          ShooterSubsystem.get().L2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
        IntakePositionSubsystem.get().new FlipIntake()
        ),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 85),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 45),
      new LoadCube(),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 130),
      ShooterSubsystem.get().L1Shot(),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 100)
    );
  }
}