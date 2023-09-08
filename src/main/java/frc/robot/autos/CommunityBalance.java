package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Utilities.TimedCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.*;

public class CommunityBalance extends SequentialCommandGroup {

  public CommunityBalance() {
    addCommands(
      new TimedCommand(new ShootCube(Constants.AUTO_L2_SHOOTER_SPEED), 0.5),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 170),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 85),
      new InstantCommand(() -> TankDriveSubsystem.get().setBrake()),
      new AutoBalance()
    );
  }
}