package frc.robot;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.autos.CommunityBalance;
import frc.robot.autos.CommunityPickupBalance;
import frc.robot.autos.JustShoot;
import frc.robot.autos.Score2;
import frc.robot.autos.ShootBalance;
import frc.robot.autos.ShootMove;
import frc.robot.autos.ThreePieceBalance;
import frc.robot.autos.TwoPieceBalanceRight;
import frc.robot.commands.EjectCube;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {
  private static final RobotContainer robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController driverHID = new CommandXboxController(0);
  private final CommandPS4Controller operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  final SendableChooser<Command> autoChooser;

  private RobotContainer() {
    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Leave community + balance", new CommunityBalance());
    autoChooser.addOption("2 Piece + Balance", new CommunityPickupBalance());
    autoChooser.addOption("Score L1 & L2", new Score2());
    autoChooser.addOption("Balance without leaving community", new ShootBalance());
    autoChooser.addOption("L2 and leave community", new ShootMove());
    autoChooser.addOption("Just Shoot", new JustShoot());
    autoChooser.addOption("2 Piece + Balance - Right", new TwoPieceBalanceRight());
    autoChooser.addOption("3 Piece + Balance UNTESTED", new ThreePieceBalance()); //TODO: Test this before CC?
    
    Shuffleboard.getTab("Driver")
      .add("Auto Chooser", autoChooser)
      .withPosition(0, 0)
      .withSize(2, 2);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // ====================================== DRIVER CONTROLS ====================================== \\

    Drive.get().setDefaultCommand(Drive.get().new TankDrive(
      () -> -driverHID.getRightY(),
      () -> -driverHID.getLeftY()));

    Trigger communityShot = driverHID.leftTrigger(Constants.TRIGGER_DEADZONE);
    communityShot.whileTrue(new ShootCube(Constants.COMMUNITY_SHOOTER_SPEED)
    );

    Trigger runIntakeForwardsTrigger = driverHID.rightTrigger(Constants.TRIGGER_DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new ParallelCommandGroup(
      new StartEndCommand(
              Intake.get()::run,
              Intake.get()::stop,
              Intake.get()),
      Indexer.get().new RunUntilBeam())
    );

    Trigger runIntakeBackwardsTrigger = driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());

    Trigger driverSlowMode = driverHID.leftBumper();
    driverSlowMode.onTrue(new InstantCommand(() -> Drive.get().slowDrive()));
    driverSlowMode.onFalse(new InstantCommand(() -> Drive.get().normalDrive()));
    
    // ====================================== Operator Controls ====================================== \\

    Trigger brakeMode = operatorHID.povUp();
    brakeMode.onTrue(new InstantCommand(() -> Drive.get().setBrake()));
    
    Trigger coastMode = operatorHID.povDown();
    coastMode.onTrue(new InstantCommand(() -> Drive.get().setCoast()));

    Trigger slowMode = operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> Drive.get().slowDrive()));
    slowMode.onFalse(new InstantCommand(() -> Drive.get().normalDrive()));

    Trigger shootL1 = operatorHID.L1();
    shootL1.whileTrue(new ShootCube(Constants.L1_SHOOTER_SPEED));

    Trigger shootL2 = operatorHID.R1();
    shootL2.whileTrue(new ShootCube(Constants.L2_SHOOTER_SPEED));

    Trigger hybridShot = operatorHID.L2();
    hybridShot.whileTrue(new ShootCube(Constants.HYBRID_SHOOTER_SPEED));

    Trigger intakeUp = operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, Constants.TRIGGER_DEADZONE);
    Trigger intakeDown = operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -Constants.TRIGGER_DEADZONE);

    intakeUp.whileTrue(Intake.get().new IntakeUpDown(true));
    intakeDown.whileTrue(Intake.get().new IntakeUpDown(false));

    Trigger clearButton = operatorHID.circle();
    clearButton.onTrue(new InstantCommand(() -> Intake.get().clearPositionStickies()));

    Trigger overrideIntake = operatorHID.square();
    overrideIntake.onTrue(new InstantCommand(() -> Indexer.get().flipUseBeamBreak()));
  }

  public static RobotContainer getInstance() {
    return robotContainer;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}