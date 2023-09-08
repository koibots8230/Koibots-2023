package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ShootCube;


public class JustShoot extends ParallelRaceGroup {
    
    public JustShoot() {
        addCommands(
            new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED),
            new WaitCommand(0.75)
        );
    }


}
