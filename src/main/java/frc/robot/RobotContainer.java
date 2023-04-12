package frc.robot;

import frc.robot.autos.*;
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
  private static final RobotContainer robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController driverHID = new CommandXboxController(0);
  private final CommandPS4Controller operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  final SendableChooser<Command> autoChooser;

  private RobotContainer() {
    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Leave community and balance", new CommunityBalance());
    autoChooser.addOption("2 Piece + Balance", new CommunityPickupBalance());
    autoChooser.addOption("Score L1 & L2", new Score2());
    autoChooser.addOption("Balance without leaving community", new ShootBalance());
    autoChooser.addOption("L2 and leave community", new ShootMove());
    autoChooser.addOption("Just Shoot", new JustShoot());
    autoChooser.addOption("2 Piece + Balance - Right", new TwoPieceBalanceRight());
    autoChooser.addOption("3 Piece + Balance UNTESTED", new ThreePieceBalance());
    
    Shuffleboard.getTab("Driver")
      .add("Auto Chooser", autoChooser)
      .withPosition(0, 0)
      .withSize(2, 2);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // ====================================== DRIVER CONTROLS ====================================== \\

    TankDriveSubsystem.get().setDefaultCommand(TankDriveSubsystem.get().new driveMotorCommand(
      () -> -driverHID.getRightY(),
      () -> -driverHID.getLeftY()));

    // Community Shot
    Trigger communityShot = driverHID.leftTrigger(Constants.TRIGGER_DEADZONE);
    communityShot.whileTrue(new ParallelCommandGroup(
      IndexerSubsystem.get().new RunIndexer(),
      ShooterSubsystem.get().CommunityShot()
      )
    );

    Trigger runIntakeForwardsTrigger = driverHID.rightTrigger(Constants.TRIGGER_DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new ParallelCommandGroup(
      IntakeSubsystem.get().new RunIntake(),
      IndexerSubsystem.get().new RunUntilBeam()
    )
    );

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());

    Trigger driverSlowMode = driverHID.leftBumper();
    driverSlowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().startSlowMode()));
    driverSlowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().stopSlowMode()));

    
    // ====================================== Operator Controls ====================================== \\

    Trigger brakeMode = operatorHID.povUp();
    brakeMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().setBrake()));
    
    Trigger coastMode = operatorHID.povDown();
    coastMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().setCoast()));

    Trigger slowMode = operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().startSlowMode()));
    slowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().stopSlowMode()));

    // Shooting
    Trigger shootL1 = operatorHID.L1();
    shootL1.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().L1Shot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger shootL2 = operatorHID.R1();
    shootL2.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().L2Shot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger hybridShot = operatorHID.L2();
    hybridShot.whileTrue(new ParallelCommandGroup(
      ShooterSubsystem.get().HybridShot(),
      IndexerSubsystem.get().new RunIndexer()));

    Trigger intakeUp = operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, Constants.TRIGGER_DEADZONE);
    Trigger intakeDown = operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -Constants.TRIGGER_DEADZONE);

    intakeUp.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(true));
    intakeDown.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(false));

    Trigger clearButton = operatorHID.circle();
    clearButton.onTrue(new InstantCommand(() -> IntakePositionSubsystem.get().ClearStickies()));

    Trigger overrideIntake = operatorHID.square();
    overrideIntake.onTrue(new InstantCommand(() -> IndexerSubsystem.get().changeUseBeamBreak()));
  }

  public static RobotContainer getInstance() {
    return robotContainer;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}