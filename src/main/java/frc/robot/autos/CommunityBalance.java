// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.*;
import com.kauailabs.navx.frc.AHRS;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommunityBalance extends SequentialCommandGroup {
  /** Creates a new CommunityBalance. */


  AHRS m_Gyro;
  public CommunityBalance() {
    addCommands(
      ShooterSubsystem.get().L2Shot(),
      TankDriveSubsystem.get().new driveDistanceCommand(0.3, 0.3, 170),
      TankDriveSubsystem.get().new driveDistanceCommand(-0.3, -0.3, 85),
      new AutoBalanceCommand()
    );
  }
}