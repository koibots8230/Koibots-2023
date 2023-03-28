package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class CommunityBalance extends SequentialCommandGroup {

  AHRS m_Gyro;
  public CommunityBalance() {
    addCommands(
      new ParallelRaceGroup(
      ShooterSubsystem.get().L2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 170),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 85),
      new AutoBalanceCommand()
    );
  }
}