package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FollowPathCommand extends CommandBase{

    private final VisionSubsystem m_VisionSubsystem;
    private final TankDriveSubsystem m_DriveSubsystem;
    
    public FollowPathCommand(VisionSubsystem _vision, TankDriveSubsystem _drive) {
        m_VisionSubsystem = _vision;
        m_DriveSubsystem = _drive;
    }

    public void execute() {
        RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_DriveSubsystem::getOdometryPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            m_DriveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_DriveSubsystem::setMotorVoltage,
            m_DriveSubsystem);
    }

}
