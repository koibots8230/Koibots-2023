package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Utilities.TimedCommand;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.TankDriveSubsystem;

public class ShootMove extends SequentialCommandGroup{
    public ShootMove() {
        addCommands(
            new TimedCommand(new Shoot(Constants.AUTO_L2_SHOOTER_SPEED), 0),
            TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 85)
        ); 
    }
}
