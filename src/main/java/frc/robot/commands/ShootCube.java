package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCube extends ParallelCommandGroup {

    public ShootCube(double speed) {
        addCommands(
            new StartEndCommand(
                    Indexer.get()::run,
                    Indexer.get()::stop,
                    Indexer.get()
            ),
            new StartEndCommand(
                    () -> Shooter.get().shoot(speed),
                    Shooter.get()::stop,
                    Shooter.get()
            )
        );
    }
}
