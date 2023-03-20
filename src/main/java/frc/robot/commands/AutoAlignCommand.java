package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoAlignCommand extends CommandBase {
    /** Creates a new ShootyCommand. */

    private Pose3d botPose;
    private Pose3d targetPose;
    private double rotationToTarget;

    private int level;

    private boolean end = false;

    private double angleDeadzone;

    public AutoAlignCommand(int _level) {
        level = _level;

        addRequirements(TankDriveSubsystem.get());
    }

    public double getAngleDeadzone(Pose3d botPose, Pose3d targetPose) {
        double angle1 = Math.atan(
                (botPose.getX() - targetPose.getX()) /
                        (Math.abs(botPose.getY() - targetPose.getY()) +
                                (Constants.AVAILABLE_CUBE_NODE_SPACE / 2)));
        double angle2 = Math.atan(
                Math.abs(botPose.getY() - targetPose.getY()) /
                        (botPose.getX() - targetPose.getX()));
        return 90 - (angle1 + angle2);
    }

    public double getRotationToTarget(Pose3d botPose, Pose3d targetPose) {
        return Math.atan(
                (botPose.getX() - targetPose.getX()) /
                (botPose.getY() - targetPose.getY()));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        botPose = ShooterSubsystem.get().getBotPose();

        Optional<Pose3d> mabyeTargetPose = ShooterSubsystem.get().getNearestTarget(botPose.getTranslation(), level);

        if (mabyeTargetPose.isEmpty()) {
            System.out.print("No target within range");
            end = true;
        }
        targetPose = mabyeTargetPose.get();

        angleDeadzone = getAngleDeadzone(botPose, targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rotationToTarget = getRotationToTarget(botPose, targetPose);
        if (rotationToTarget > 0) {
            TankDriveSubsystem.get().setMotor(Constants.ALIGN_DRIVE_SPEED, -Constants.ALIGN_DRIVE_SPEED);
        } else if (rotationToTarget < 0) {
            TankDriveSubsystem.get().setMotor(-Constants.ALIGN_DRIVE_SPEED, Constants.ALIGN_DRIVE_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        TankDriveSubsystem.get().setMotor(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(rotationToTarget) < angleDeadzone) {
            System.out.print("Aligned to target");
            return true;
        }
        return end;
    }
}