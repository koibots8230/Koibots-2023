package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

public class ShootMove extends SequentialCommandGroup{
    public ShootMove() {
        addCommands(
            ShooterSubsystem.get().AutoL2Shot(),
            TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 85)
        ); 
    }
}
