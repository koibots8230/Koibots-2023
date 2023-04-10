package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceThenShoot;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class CommunityPickupBalance extends SequentialCommandGroup {
  AHRS m_Gyro;
  public CommunityPickupBalance() {
    addCommands(
      new ParallelRaceGroup(ShooterSubsystem.get().AutoL2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 150),
      new ParallelRaceGroup(TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.435, 65), new LoadCube()),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.435, 55),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 80),
      new InstantCommand(() -> TankDriveSubsystem.get().setBrake()),
      // new ParallelRaceGroup(ShooterSubsystem.get().CommunityShot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(.5)),
      new AutoBalanceThenShoot()
    );
  }
}