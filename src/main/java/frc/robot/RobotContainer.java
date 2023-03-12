package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Enumeration;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem.CommunityShotCommand;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Commands
  private final TankDriveSubsystem.driveMotorCommand m_driveCommand = TankDriveSubsystem.get().new driveMotorCommand(
      () -> m_driverHID.getRightY(),
      () -> m_driverHID.getLeftY());

  // Shuffleboard
  SendableChooser<PathPlannerTrajectory> m_autoChooser;

  GenericEntry autobal_leftSpeed; // = m_autotab.add("autobal leftSpeed", Constants.AUTO_LEFT_SPEED).getEntry();
  GenericEntry autobal_rightSpeed; // = m_autotab.add("autobal rightSpeed", Constants.AUTO_RIGHT_SPEED).getEntry();
  GenericEntry shoot_leftSpeed; // = m_autotab.add("shoot leftSpeed", Constants.SHOOT_LEFT_SPEED).getEntry();
  GenericEntry shoot_rightSpeed;// = m_autotab.add("shoot rightSpeed", Constants.SHOOT_RIGHT_SPEED).getEntry();
  GenericEntry shoot_time; // = m_autotab.add("Shoot time", Constants.SHOOT_SECONDS).getEntry();
  GenericEntry autobal_limit;
  GenericEntry shoot_limit;


  private RobotContainer() {
    // choosing what auto
    m_autoChooser = new SendableChooser<PathPlannerTrajectory>();

    Enumeration<String> PathNames = Constants.Paths.keys();
    while (PathNames.hasMoreElements()) {
      String key = PathNames.nextElement();

      m_autoChooser.addOption(key, Constants.Paths.get(key));
    }

    ShuffleboardTab m_autotab = Shuffleboard.getTab("Auto");
    m_autoChooser.addOption(("NO AUTO"), null);

    ShuffleboardTab m_shuffleboard = Shuffleboard.getTab("Main");
    m_shuffleboard.add(m_autoChooser);
    m_autotab.add(m_autoChooser);
    m_shuffleboard.addNumber("Encoder Left", () -> TankDriveSubsystem.get().getEncoderPositions()[0]);
    m_shuffleboard.addNumber("Encoder Right", () -> TankDriveSubsystem.get().getEncoderPositions()[1]);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    TankDriveSubsystem.get().setDefaultCommand(m_driveCommand);

    // Create Triggers here | Triggers should be named t_CommandName

    // ======================================Operator Controls====================================== \\

    Trigger slowMode = m_operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().SlowDrive()));
    slowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().UnslowDrive()));

    // Shooting
    Trigger shootL2 = m_operatorHID.L1();
    Trigger shootL3 = m_operatorHID.R1();

    shootL2.whileTrue(ShooterSubsystem.get().new LevelShootCommand(2));
    shootL3.whileTrue(ShooterSubsystem.get().new LevelShootCommand(3));


    Trigger clearButton = m_operatorHID.circle();
    clearButton.whileTrue(new InstantCommand(() -> IntakeSubsystem.getIntakeSubsystem().ClearStickies(), IntakeSubsystem.getIntakeSubsystem()));

    // ======================================DRIVER
    // CONTROLS======================================
    // create commands
    // 5 = left bumper
    // 6 = right bumper

    // Community Shot
    Trigger shootTrigger = m_driverHID.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, Constants.DEADZONE);
    CommunityShotCommand com_shot_cmd = ShooterSubsystem.get().new CommunityShotCommand();
    shootTrigger.whileTrue(com_shot_cmd);

    // Flip Intake
    // Trigger flipTrigger = m_driverHID.leftBumper();
    // flipTrigger.onTrue(m_intake.new FlipIntake(m_intake));\

    Trigger runIntakeForwardsTrigger = m_driverHID.rightTrigger(Constants.DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new LoadCube());

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  public CommandGenericHID getController() {
    return m_driverHID;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new FollowPathWithEvents(
      TankDriveSubsystem.get().followTrajectoryCommand(
        m_autoChooser.getSelected()), 
      m_autoChooser.getSelected().getMarkers(), 
      Constants.Events);
  }
  
}