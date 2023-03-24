// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommunityPickupBalance extends SequentialCommandGroup {
  /** Creates a new CommunityBalance. */

  AHRS m_Gyro;
  public CommunityPickupBalance() {
    addCommands(
      new ParallelRaceGroup(ShooterSubsystem.get().L2Shot(), IndexerSubsystem.get().new RunIndexer(), new WaitCommand(0.5)),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 160),
      new ParallelRaceGroup(TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.4, 20), new LoadCube()),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.4, 20),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 80),
      new AutoBalanceCommand()
    );
  }
}