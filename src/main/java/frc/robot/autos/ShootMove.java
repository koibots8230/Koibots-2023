package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.utilities.TimedCommand;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.drive.Drive;

public class ShootMove extends SequentialCommandGroup{
    public ShootMove() {
        addCommands(
            new TimedCommand(new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED), 0),
            Drive.get().new driveDistanceCommand(0.3, 0.3, 85)
        ); 
    }
}
