package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class JustShoot extends ParallelRaceGroup {
    
    public JustShoot() {
        addCommands(
            ShooterSubsystem.get().AutoL2Shot(),
            IndexerSubsystem.get().new RunIndexer(),
            new WaitCommand(0.75)
        );
    }


}
