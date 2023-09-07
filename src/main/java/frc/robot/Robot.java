package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilities.NAVX;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    @SuppressWarnings("resource")
    public void robotInit() {

        m_robotContainer = RobotContainer.getInstance();
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        NAVX.get().zeroYaw();
        TankDriveSubsystem.get().resetEncoders();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

        ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");

        debugTab
            .addDoubleArray("Drive Voltages", TankDriveSubsystem.get()::getVoltages)
            .withWidget("Graph")
            .withPosition(0, 0)
            .withSize(3, 3);

        debugTab
            .addDouble("Yaw", NAVX.get()::getAngle)
            .withPosition(3, 0)
            .withSize(3, 3);
        
        debugTab
            .addDouble("Pitch", NAVX.get()::getRoll)
            .withWidget("Gyro")
            .withPosition(6, 0)
            .withSize(3, 3);

        debugTab
            .addDouble("Roll", NAVX.get()::getPitch)
            .withWidget("Gyro")
            .withPosition(9, 0)
            .withSize(3, 3);

        debugTab
            .addDoubleArray("Encoders", TankDriveSubsystem.get()::getEncoderPositions)
            .withWidget("Graph")
            .withPosition(0, 3)
            .withSize(3, 3);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        NAVX.get().zeroYaw();
        TankDriveSubsystem.get().resetEncoders();
        TankDriveSubsystem.get().setCoast();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        TankDriveSubsystem.get().resetEncoders();
    
    }

    @Override
    public void teleopInit() {
        TankDriveSubsystem.get().setCoast();
        IntakePositionSubsystem.get().setCoast();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        TankDriveSubsystem.get().setBrake();
        IntakePositionSubsystem.get().setBrake();
    }

    @Override
    public void testInit() {
        IntakePositionSubsystem.get().setCoast();
        TankDriveSubsystem.get().setCoast();
        CommandScheduler.getInstance().cancelAll();
        System.out.println("Reset to Coast");
        DriverStationDataJNI.setEnabled(true);
    }

    @Override
    public void testPeriodic() {
    }
}
