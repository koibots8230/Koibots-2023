package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.Enumeration;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  SendableChooser<List<PathPlannerTrajectory>> m_autoChooser;

  private RobotContainer() {
    // choosing what auto
    m_autoChooser = new SendableChooser<List<PathPlannerTrajectory>>();

    Enumeration<String> PathNames = Constants.PATHS.keys();
    while (PathNames.hasMoreElements()) {
      String key = PathNames.nextElement();

      m_autoChooser.addOption(key, Constants.PATHS.get(key));
    }

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // ====================================== Operator Controls ====================================== \\

    Trigger slowMode = m_operatorHID.triangle();
    slowMode.onTrue(new InstantCommand(() -> TankDriveSubsystem.get().SlowDrive()));
    slowMode.onFalse(new InstantCommand(() -> TankDriveSubsystem.get().UnslowDrive()));

    // Shooting
    Trigger shootL1 = m_operatorHID.L1();
    shootL1.whileTrue(ShooterSubsystem.get().L1Shot());

    Trigger shootL2 = m_operatorHID.R1();
    shootL2.whileTrue(ShooterSubsystem.get().L2Shot());



    
    Trigger clearButton = m_operatorHID.circle();
    clearButton.whileTrue(new InstantCommand(() -> IntakeSubsystem.getIntakeSubsystem().ClearStickies(), IntakeSubsystem.getIntakeSubsystem()));

    // ====================================== DRIVER CONTROLS ====================================== \\
    // create commands
    // 5 = left bumper
    // 6 = right bumper

    TankDriveSubsystem.get().setDefaultCommand(TankDriveSubsystem.get().new driveMotorCommand(
      () -> m_driverHID.getRightY(),
      () -> m_driverHID.getLeftY()));

    // Community Shot
    Trigger shootHybrid = m_driverHID.leftTrigger(Constants.TRIGGER_DEADZONE);
    shootHybrid.whileTrue(ShooterSubsystem.get().HybridShot());

    // Flip Intake
    // Trigger flipTrigger = m_driverHID.leftBumper();
    // flipTrigger.onTrue(m_intake.new FlipIntake(m_intake));\

    Trigger runIntakeForwardsTrigger = m_driverHID.rightTrigger(Constants.TRIGGER_DEADZONE);
    runIntakeForwardsTrigger.whileTrue(new LoadCube());

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RamseteAutoBuilder(
      TankDriveSubsystem.get()::getRobotPose,
      TankDriveSubsystem.get()::resetOdometry,
      new RamseteController(),
      new DifferentialDriveKinematics(Constants.ROBOT_WIDTH_m),
      Constants.PP_FEED_FORWARD,
      TankDriveSubsystem.get()::getWheelSpeeds,
      Constants.AUTO_PID,
      TankDriveSubsystem.get()::setVoltage,
      Constants.EVENTS,
      false,
      TankDriveSubsystem.get()
    ).fullAuto(m_autoChooser.getSelected());
  }
}