package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCube extends ParallelCommandGroup {

    public ShootCube(double speed) {
        addCommands(
            IndexerSubsystem.get().new RunIndexer(),
            ShooterSubsystem.get().new Shoot(speed)
        );
    }
}
