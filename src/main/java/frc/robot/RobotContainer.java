package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.networktables.GenericEntry;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.CommunityShotCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Subsystems
  private final TankDriveSubsystem m_tankDriveSubsystem = new TankDriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final MiscDashboardSubsystem m_miscDashboardSubsystem = new MiscDashboardSubsystem(m_intake, m_ShooterSubsystem, m_tankDriveSubsystem);

  // Controlers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Commands
  private final TankDriveSubsystem.driveMotorCommand m_driveCommand = m_tankDriveSubsystem.new driveMotorCommand(
      () -> m_driverHID.getRightY(),
      () -> m_driverHID.getLeftY(),
      m_tankDriveSubsystem);

  // Shuffleboard
  SendableChooser<Command> m_autoChooser;

  GenericEntry autobal_leftSpeed; // = m_autotab.add("autobal leftSpeed", Constants.AUTO_LEFT_SPEED).getEntry();
  GenericEntry autobal_rightSpeed; // = m_autotab.add("autobal rightSpeed", Constants.AUTO_RIGHT_SPEED).getEntry();
  GenericEntry shoot_leftSpeed; // = m_autotab.add("shoot leftSpeed", Constants.SHOOT_LEFT_SPEED).getEntry();
  GenericEntry shoot_rightSpeed;// = m_autotab.add("shoot rightSpeed", Constants.SHOOT_RIGHT_SPEED).getEntry();
  GenericEntry shoot_time; // = m_autotab.add("Shoot time", Constants.SHOOT_SECONDS).getEntry();
  GenericEntry autobal_limit;
  GenericEntry shoot_limit;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private void configureButtonBindings() {
    m_tankDriveSubsystem.setDefaultCommand(m_driveCommand);

    // Create Triggers here | Triggers should be named t_CommandName

    // ======================================Operator Controls====================================== \\

    Trigger slowMode = m_operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> m_tankDriveSubsystem.SlowDrive()));
    slowMode.onFalse(new InstantCommand(() -> m_tankDriveSubsystem.UnslowDrive()));

    // Speed Up/Down
    // Trigger operatorSpeedUp = m_operatorHID.cross();
    // Trigger operatorSpeedDown = m_operatorHID.circle();

    // operatorSpeedUp.onTrue(new setSpeedCommand(true, m_tankDriveSubsystem));
    // operatorSpeedDown.onTrue(new setSpeedCommand(false, m_tankDriveSubsystem));

    // LED
    // Trigger leftTrigger_op =
    // m_operatorHID.axisGreaterThan(PS4Controller.Axis.kL2.value,
    // Constants.DEADZONE);
    // Trigger rightTrigger_op =
    // m_operatorHID.axisGreaterThan(PS4Controller.Axis.kR2.value,
    // Constants.DEADZONE);

    // Shooting
    Trigger shootL2 = m_operatorHID.L1();
    Trigger shootL3 = m_operatorHID.R1();

    shootL2.whileTrue(m_ShooterSubsystem.new LevelShootCommand(m_ShooterSubsystem, 2));
    shootL3.whileTrue(m_ShooterSubsystem.new LevelShootCommand(m_ShooterSubsystem, 3));

    // Manual Intake Up/Down
    Trigger intakeMoveUp = m_operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -.3);
    Trigger intakeMoveDown = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, .3);
    // intakeMoveUp.whileTrue(new IntakeManualCommand(m_intake, true));
    // intakeMoveDown.whileTrue(new IntakeManualCommand(m_intake, false));

    Trigger clearButton = m_operatorHID.circle();

    clearButton.whileTrue(new InstantCommand(() -> m_intake.ClearStickies(), m_intake));

    // ======================================DRIVER
    // CONTROLS======================================
    // create commands
    // 5 = left bumper
    // 6 = right bumper

    // Community Shot
    Trigger shootTrigger = m_driverHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, Constants.DEADZONE);
    CommunityShotCommand com_shot_cmd = m_ShooterSubsystem.new CommunityShotCommand(m_ShooterSubsystem);
    shootTrigger.whileTrue(com_shot_cmd);

    // Flip Intake
    // Trigger flipTrigger = m_driverHID.leftBumper();
    // flipTrigger.onTrue(m_intake.new FlipIntake(m_intake));

    Trigger runIntakeForwardsTrigger = m_driverHID.rightTrigger(Constants.DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new IntakeCommand(m_intake, true));

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new IntakeCommand(m_intake, false)
        .alongWith(Commands.runEnd(
            () -> m_ShooterSubsystem.SetShooter(-.1),
            () -> m_ShooterSubsystem.SetShooter(0),
            m_ShooterSubsystem)));

    // Slow Shooter
    Trigger runInShooterSlowly = m_operatorHID.square();
    runInShooterSlowly.whileTrue(new IntakeCommand(m_intake, true)
        .alongWith(Commands.runEnd(
            () -> m_ShooterSubsystem.SetShooter(.2),
            () -> m_ShooterSubsystem.SetShooter(0),
            m_ShooterSubsystem)));

  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    // choosing what auto
    m_autoChooser = new SendableChooser<Command>();

    configureButtonBindings();
  }

  public CommandGenericHID getController() {
    return m_driverHID;
  }

  public void ResetPositions() {
    m_intake.resetPosition();
  }

  public TankDriveSubsystem getDrive() {
    return m_tankDriveSubsystem;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}