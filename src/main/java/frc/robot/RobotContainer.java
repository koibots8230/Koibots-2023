package frc.robot;

import frc.robot.autos.CommunityBalance;
import frc.robot.autos.CommunityPickupBalance;
import frc.robot.autos.Score2;
import frc.robot.autos.ShootBalance;
import frc.robot.autos.ShootMove;
import frc.robot.autos.ThreePieceBalance;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  SendableChooser<Command> m_autoChooser;

  private RobotContainer() {
    m_autoChooser = new SendableChooser<Command>();

    m_autoChooser.setDefaultOption("Leave community and balance", new CommunityBalance());
    m_autoChooser.addOption("2 Piece + Balance", new CommunityPickupBalance());
    m_autoChooser.addOption("Score L1 & L2", new Score2());
    m_autoChooser.addOption("Balance without leaving community", new ShootBalance());
    m_autoChooser.addOption("L2 and leave community", new ShootMove());
    
    Shuffleboard.getTab("Driver")
      .add("Auto Chooser", m_autoChooser)
      .withPosition(0, 0)
      .withSize(2, 2);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // ====================================== DRIVER CONTROLS ====================================== \\

    TankDriveSubsystem.get().setDefaultCommand(TankDriveSubsystem.get().new driveMotorCommand(
      () -> -m_driverHID.getRightY(),
      () -> -m_driverHID.getLeftY()));

    // Community Shot
    Trigger communityShot = m_driverHID.leftTrigger(Constants.TRIGGER_DEADZONE);
    communityShot.whileTrue(new ParallelCommandGroup(
      IndexerSubsystem.get().new RunIndexer(),
      ShooterSubsystem.get().CommunityShot()
      )
    );

    Trigger runIntakeForwardsTrigger = m_driverHID.rightTrigger(Constants.TRIGGER_DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new ParallelCommandGroup(
      IntakeSubsystem.get().new RunIntake(),
      IndexerSubsystem.get().new RunUntilBeam()
    )
    );

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());

    Trigger driverSlowMode = m_driverHID.leftBumper();
    driverSlowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().SlowDrive()));
    driverSlowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().UnslowDrive()));

    
    // ====================================== Operator Controls ====================================== \\

    Trigger brakeMode = m_operatorHID.povUp();
    brakeMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().setBrake()));
    
    Trigger coastMode = m_operatorHID.povDown();
    coastMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().setCoast()));

    Trigger slowMode = m_operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().SlowDrive()));
    slowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().UnslowDrive()));

    // Shooting
    Trigger shootL1 = m_operatorHID.L1();
    shootL1.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().L1Shot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger shootL2 = m_operatorHID.R1();
    shootL2.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().L2Shot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger hybridShot = m_operatorHID.L2();
    hybridShot.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().HybriShot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger intakeUp = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, Constants.TRIGGER_DEADZONE);
    Trigger intakeDown = m_operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -Constants.TRIGGER_DEADZONE);

    intakeUp.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(true));
    intakeDown.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(false));

    Trigger clearButton = m_operatorHID.circle();
    clearButton.onTrue(new InstantCommand(() -> IntakePositionSubsystem.get().ClearStickies()));

    Trigger overrideIntake = m_operatorHID.square();
    overrideIntake.onTrue(new InstantCommand(() -> IndexerSubsystem.get().changeUseBeamBreak()));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}