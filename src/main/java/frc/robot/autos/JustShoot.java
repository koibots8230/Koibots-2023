package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class JustShoot extends ParallelRaceGroup {
    
    public JustShoot() {
        addCommands(
            ShooterSubsystem.get().AutoL2Shot(),
            IndexerSubsystem.get().new RunIndexer(),
            new WaitCommand(0.75)
        );
    }


}
