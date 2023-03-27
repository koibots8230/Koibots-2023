package frc.robot;

import frc.robot.Utilities.TimedCommand;
import frc.robot.autos.CommunityBalance;
import frc.robot.autos.CommunityPickupBalance;
import frc.robot.autos.Score2;
import frc.robot.autos.ShootBalance;
import frc.robot.autos.ShootMove;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.Enumeration;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

public class RobotContainer {
  private static RobotContainer m_robotContainer = new RobotContainer();

  // Controllers
  private final CommandXboxController m_driverHID = new CommandXboxController(0);
  private final CommandPS4Controller m_operatorHID = new CommandPS4Controller(1);

  // Shuffleboard
  SendableChooser<String> m_pathChooser;
  SendableChooser<Command> m_autoChooser;

  //LED system 
  private final LEDsystem LEDstrips = new LEDsystem(Constants.LEDPort);//addressable LED only works from one port.

  private RobotContainer() {
    m_autoChooser = new SendableChooser<Command>();

    m_autoChooser.setDefaultOption("Leave community and balance", new CommunityBalance());
    m_autoChooser.addOption("Leave Community and Balance + Piece Pickup", new CommunityPickupBalance());
    m_autoChooser.addOption("Score L1 & L2", new Score2());
    m_autoChooser.addOption("Balance without leaving community", new ShootBalance());
    m_autoChooser.addOption("L2 and leave community", new ShootMove());

    m_pathChooser = new SendableChooser<String>();

    // Paths are in Constants
    Enumeration<String> PathNames = Constants.PATHS.keys();
    while (PathNames.hasMoreElements()) {
      String key = PathNames.nextElement();
      m_pathChooser.addOption(key, Constants.PATHS.get(key));
    }
    
    Shuffleboard.getTab("Driver")
      .add("Auto Chooser", m_autoChooser)
      .withPosition(0, 0)
      .withSize(2, 2);

    Shuffleboard.getTab("Driver")
      .add("Path Chooser", m_pathChooser)
      .withPosition(2, 0)
      .withSize(2, 2);

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
    // 6 = right bumperm

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
    runIntakeForwardsTrigger.whileTrue(new ParallelRaceGroup(
      new SequentialCommandGroup(
        new TimedCommand(new IndexerSubsystem().new RunIndexer(), 0.5),
        IndexerSubsystem.get().new RunUntilBeam()),
      IntakeSubsystem.getIntakeSubsystem().new RunIntake()
    ));

    // Reverse Intake/Midtake/Shooter
    Trigger runIntakeBackwardsTrigger = m_driverHID.rightBumper();
    runIntakeBackwardsTrigger.whileTrue(new EjectCube());
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }
  public LEDsystem getLEDs(){
    return LEDstrips;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_pathChooser.getSelected() != null && m_pathChooser.getSelected() != "Legacy") {
      /*
      return new FollowPathWithEvents(
      new RamseteAutoBuilder(
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
      ).followPath(PathPlanner.loadPath(m_pathChooser.getSelected(), Constants.AUTO_CONSTRAINTS))
      , null
      , null); */
      PathPlannerTrajectory path = PathPlanner.loadPath(m_pathChooser.getSelected(), Constants.AUTO_CONSTRAINTS);
      TankDriveSubsystem.get().resetOdometry(path.getInitialPose());
      return new FollowPathWithEvents(
        new PathFollower(
          path, 
          TankDriveSubsystem.get()::getRobotPose, 
          new RamseteController(),
          Constants.PP_FEED_FORWARD,
          new DifferentialDriveKinematics(Constants.ROBOT_WIDTH_m),
          TankDriveSubsystem.get()::getWheelSpeeds,
          new PIDController(.00075, 0, 0),
          new PIDController(.00075, 0, 0),
          TankDriveSubsystem.get()::setVoltage,
          TankDriveSubsystem.get()
          ),
        path.getMarkers(), 
        Constants.EVENTS);
    }
    return m_autoChooser.getSelected();
  }
}