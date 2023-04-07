package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceThenShoot;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;

public class ThreePieceBalance extends SequentialCommandGroup {
  AHRS m_Gyro;
  double mainSpeed = 0.3;
  double curveSpeed = 0.42;
  public ThreePieceBalance() {
    addCommands(
      new ParallelRaceGroup(ShooterSubsystem.get().AutoL2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(mainSpeed, mainSpeed, 150),
      new ParallelRaceGroup(TankDriveSubsystem.get().new driveDistanceCommand(mainSpeed, curveSpeed, 65), new LoadCube()),
      TankDriveSubsystem.get().new driveDistanceCommand(-mainSpeed, -curveSpeed, 55),
      new ParallelRaceGroup(
        TankDriveSubsystem.get().new driveDistanceCommand(-mainSpeed, -mainSpeed, 40),
        ShooterSubsystem.get().CommunityShot(),
        IndexerSubsystem.get().new RunIndexer()),
      TankDriveSubsystem.get().new driveDistanceCommand(mainSpeed, mainSpeed, 40),
      new ParallelRaceGroup(TankDriveSubsystem.get().new driveDistanceCommand(curveSpeed, mainSpeed, 65), new LoadCube()),
      TankDriveSubsystem.get().new driveDistanceCommand(-curveSpeed, -mainSpeed, 55),
      TankDriveSubsystem.get().new driveDistanceCommand(-mainSpeed, -mainSpeed, 80),
      new InstantCommand(() -> TankDriveSubsystem.get().setBrake()),
      // new ParallelRaceGroup(ShooterSubsystem.get().CommunityShot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(.5)),
      new AutoBalanceThenShoot()
    );
  }
}