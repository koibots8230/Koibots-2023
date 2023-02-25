/* RobotBuilder Version: 5.0

This file was generated by RobotBuilder. It contains sections of
code that are automatically generated and assigned by robotbuilder.
These sections will be updated in the future when you export to
Java from RobotBuilder. Do not put any code or make any change in
the blocks indicating autogenerated code or it will be lost on an
update. Deleting the comments indicating the section will prevent
it from being updated in the future.

ROBOTBUILDER TYPE: RobotContainer. */

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.SwitchIntakeDirection;
import frc.robot.subsystems.TankDriveSubsystem.SwitchDrivetrainInvert;
import frc.robot.commands.IntakeCommand;

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
  public final TankDriveSubsystem m_tankDriveSubsystem = new TankDriveSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static MiscDashboardSubsystem m_miscDashboardSubsystem = new MiscDashboardSubsystem();

  // other stuff
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  private DoubleSupplier leftDriveTrain = () -> m_driverHID.getLeftY();
  private DoubleSupplier rightDriveTrain = () -> m_driverHID.getRightY();

  private final TankDriveSubsystem.driveMotorCommand m_driveCommand = m_tankDriveSubsystem.new driveMotorCommand(
      rightDriveTrain,
      leftDriveTrain,
      m_tankDriveSubsystem);


  // Drivetrain is reversed when button A is pressed on the controller:
  SwitchDrivetrainInvert m_SwitchDrivetrainInvertCommand = m_tankDriveSubsystem.new SwitchDrivetrainInvert(
      m_tankDriveSubsystem);


  SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  SendableChooser<String> m_driverChooser = new SendableChooser<>();

  // m_controllerType 0 -> Unrecognized
  // m_controllerType 1 -> Xbox Controller
  // m_controllerType 2 -> Playstation Controller
  // m_controllerType 3 -> Flight Joystick
  int m_controllerType;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_tankDriveSubsystem.setDefaultCommand(m_driveCommand);

    // ==================OPERATOR CONTROLS======================================

    // Create Triggers here | Triggers should be named t_CommandName
    Trigger leftTrigger = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kL2.value, Constants.DEADZONE);
    Trigger rightTrigger = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kR2.value, Constants.DEADZONE);


    Trigger operatorSpeedUp = m_operatorHID.cross();
    Trigger operatorSpeedDown = m_operatorHID.circle();
    operatorSpeedUp.onTrue(new setSpeedCommand(true, m_tankDriveSubsystem));
    operatorSpeedDown.onTrue(new setSpeedCommand(false, m_tankDriveSubsystem));

    Trigger intakeMoveUp = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, Constants.DEADZONE);
    Trigger intakeMoveDown = m_operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -Constants.DEADZONE);
    intakeMoveUp.whileTrue(new InstantCommand(() -> m_intake.setRaiseIntakeSpeed(0.1), m_intake));
    intakeMoveDown.whileTrue(new InstantCommand(() -> m_intake.setRaiseIntakeSpeed(-0.1), m_intake));
    intakeMoveUp.or(intakeMoveDown).onFalse(new InstantCommand(() -> m_intake.setRaiseIntakeSpeed(0), m_intake));



    // ================DRIVER CONTROLS==========================================
    // create commands
    // 5 = left bumper
    // 6 = right bumper

    // Intake is toggled when left bumper is pressed
    Trigger flipTrigger = m_driverHID.leftBumper();
    flipTrigger.onTrue(m_intake.new FlipIntake(m_intake));

    // Intake runs FORWARD when right trigger is pressed
    Trigger runIntakeForwardsTrigger = m_driverHID.rightTrigger(Constants.DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new IntakeCommand(m_intake, true));

    // Intake runs BACKWARD when right bumper is pressed
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new IntakeCommand(m_intake, false));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  public CommandGenericHID getController() {
    return m_driverHID;
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
