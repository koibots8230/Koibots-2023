package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Utilities.TimedCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;

public class ShootBalance extends SequentialCommandGroup {
  public ShootBalance() {
    addCommands(
      new TimedCommand(new Shoot(Constants.AUTO_L2_SHOOTER_SPEED), 0.5),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 85),
      new AutoBalance()
    );
  }
}