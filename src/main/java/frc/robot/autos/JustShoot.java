package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shoot;


public class JustShoot extends ParallelRaceGroup {
    
    public JustShoot() {
        addCommands(
            new Shoot(Constants.AUTO_L2_SHOOTER_SPEED),
            new WaitCommand(0.75)
        );
    }


}
