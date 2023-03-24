package frc.robot;

import frc.robot.autos.CommunityBalance;
import frc.robot.autos.CommunityPickupBalance;
import frc.robot.autos.Score2;
import frc.robot.autos.ShootBalance;
import frc.robot.autos.ShootMove;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.Enumeration;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  SendableChooser<String> m_pathChooser;
  SendableChooser<Command> m_autoChooser;

  private RobotContainer() {
    m_autoChooser = new SendableChooser<Command>();

    m_autoChooser.setDefaultOption("Leave community and balance", new CommunityBalance());
    m_autoChooser.addOption("Leave Community and Balance + Piece Pickup", new CommunityPickupBalance());
    m_autoChooser.addOption("Score L1 & L2", new Score2());
    m_autoChooser.addOption("Balance without leaving community", new ShootBalance());
    m_autoChooser.addOption("L2 and leave community", new ShootMove());

    SmartDashboard.putData(m_autoChooser);

    // choosing what auto
    m_pathChooser = new SendableChooser<String>();

    Enumeration<String> PathNames = Constants.PATHS.keys();
    while (PathNames.hasMoreElements()) {
      String key = PathNames.nextElement();

      m_pathChooser.addOption(key, Constants.PATHS.get(key));
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
    shootL1.whileTrue(ShooterSubsystem.get().L1Shot())
    .whileTrue(IndexerSubsystem.get().new RunIndexer());

    Trigger shootL2 = m_operatorHID.R1();
    shootL2.whileTrue(ShooterSubsystem.get().L2Shot())
    .whileTrue(IndexerSubsystem.get().new RunIndexer());


    Trigger intakeUp = m_operatorHID.axisGreaterThan(PS4Controller.Axis.kLeftY.value, Constants.TRIGGER_DEADZONE);
    Trigger intakeDown = m_operatorHID.axisLessThan(PS4Controller.Axis.kLeftY.value, -Constants.TRIGGER_DEADZONE);

    intakeUp.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(true));
    intakeDown.whileTrue(IntakePositionSubsystem.get().new IntakeUpDown(false));

    
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
    Trigger communityShot = m_driverHID.leftTrigger(Constants.TRIGGER_DEADZONE);
    communityShot.whileTrue(new ParallelCommandGroup(
      IndexerSubsystem.get().new RunIndexer(),
      ShooterSubsystem.get().CommunityShot()
      )
    );
    // Flip Intake
    Trigger flipTrigger = m_driverHID.leftBumper();
    flipTrigger.onTrue(IntakePositionSubsystem.get().new FlipIntake());

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
    ).fullAuto(PathPlanner.loadPathGroup(m_pathChooser.getSelected(), Constants.AUTO_CONSTRAINTS));
  }
}