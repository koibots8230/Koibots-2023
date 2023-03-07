// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.kauailabs.navx.frc.AHRS;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CommunityBalance extends SequentialCommandGroup {
  /** Creates a new CommunityBalance. */
  TankDriveSubsystem m_tankDriveSubsystem;
  ShooterSubsystem m_ShooterSubsystem;
  IntakeSubsystem m_IntakeSubsystem;

  double m_EncoderDistance;
  double m_ShootTime;

  double m_leftSpeed;
  double m_rightSpeed;

  AHRS m_Gyro;
  public CommunityBalance(
    TankDriveSubsystem tankDriveSubsystem,
      ShooterSubsystem shooterSubsystem,
      double ShootTime,
      double EncoderDistance,
      double leftSpeed,
      double rightSpeed,
      AHRS Gyro,
      IntakeSubsystem intakeSubsystem
  ) {

    m_tankDriveSubsystem = tankDriveSubsystem;
    m_ShooterSubsystem = shooterSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_EncoderDistance = EncoderDistance;
    m_ShootTime = ShootTime;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> m_IntakeSubsystem.turnOn(true), m_IntakeSubsystem),
      m_ShooterSubsystem.new ShootTimeCommand(shooterSubsystem, ShootTime),
      new InstantCommand(() -> m_IntakeSubsystem.turnOff(), m_IntakeSubsystem),
      m_tankDriveSubsystem.new driveDistanceCommand(m_leftSpeed, m_rightSpeed, 170, tankDriveSubsystem),
      m_tankDriveSubsystem.new driveDistanceCommand(-m_leftSpeed, -m_rightSpeed, 85, tankDriveSubsystem),
      new InstantCommand(() -> m_tankDriveSubsystem.setBrake()),
      new AutoBalanceCommand(Gyro, tankDriveSubsystem)
    );
  }
}
