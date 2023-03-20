package frc.robot.commands;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends ParallelRaceGroup {
    /** Creates a new ShootyCommand. */
    private int level;

    public AutoShootCommand(int _level) {
        addRequirements(ShooterSubsystem.get());
        level = _level;

        addCommands(new AutoShoot(), new WaitCommand(2));
    }

    public class AutoShoot extends CommandBase {

        private double distance2d;
        private double heightDifference;

        private double Velocity;

        private boolean end = false;

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {

            Pose3d botPose = ShooterSubsystem.get().getBotPose();
            Optional<Pose3d> mabyeTargetPose = ShooterSubsystem.get().getNearestTarget(botPose.getTranslation(), level);

            if (mabyeTargetPose.isEmpty()) {
                end = true;
            }

            Pose3d targetPose = mabyeTargetPose.get();

            distance2d = botPose.getTranslation().toTranslation2d()
                    .getDistance(targetPose.getTranslation().toTranslation2d());

            heightDifference = targetPose.getZ() - Constants.SHOOTER_FROM_GROUND - botPose.getZ();

            Velocity = Math.sqrt(
                    (-Constants.GRAVITY * Math.pow(distance2d, 2)) /
                            (Math.pow(Math.cos(Constants.SHOOTER_ANGLE), 2) *
                                    (heightDifference - (distance2d * Math.tan(Constants.SHOOTER_ANGLE)))));

            ShooterSubsystem.get().SetShooterVelocity(Velocity);

            ShuffleboardTab shootTab = Shuffleboard.getTab("Shooting");

            shootTab.addNumber("Distance To Target", () -> distance2d).withPosition(0, 0).withSize(2, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Height Difference With Target", () -> heightDifference).withPosition(0, 1)
                    .withSize(2, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Calculated Velocity", () -> Velocity).withPosition(0, 2).withSize(2, 1)
                    .withWidget(BuiltInWidgets.kTextView);

            shootTab.addNumber("Target X", () -> targetPose.getX()).withPosition(2, 0).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Target Y", () -> targetPose.getY()).withPosition(2, 1).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Target Z", () -> targetPose.getZ()).withPosition(2, 2).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);

            shootTab.addNumber("Bot X", () -> botPose.getX()).withPosition(3, 0).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Bot Y", () -> botPose.getY()).withPosition(3, 1).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);
            shootTab.addNumber("Bot Z", () -> botPose.getZ()).withPosition(3, 2).withSize(1, 1)
                    .withWidget(BuiltInWidgets.kTextView);

        }

        @Override
        public void end(boolean interrupted) {
            ShooterSubsystem.get().SetShooterVelocity(0);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return end;
        }
    }
}