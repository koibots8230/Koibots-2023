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
        
    }

}
