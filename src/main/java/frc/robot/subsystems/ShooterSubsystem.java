package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    int ShootLevel = 2;
    final Optional<EstimatedRobotPose> BotThing = VisionSubsystem.photonPoseEstimator.update();
    final Pose3d Bot3d = BotThing.get().estimatedPose;
    int Closest = 0;
    double ClosestDistance = 0;
    Translation2d Spot = new Translation2d(1, 1);
    for (int a = 0; a < 6; a++) {
      if (ShootLevel == 2) {
      Spot = Constants.MIDDLE_SPOTS.get(a).toTranslation2d();
    } else {
      Spot = Constants.HIGH_SPOTS.get(a).toTranslation2d();
    }
    double xDistance = Bot3d.getX() - Spot.getX();
    double yDistance = Bot3d.getY() - Spot.getY();
    double distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
    if (distance < ClosestDistance) {
      ClosestDistance = distance;
      Closest = a;
    }}
    Translation3d ShootingSpot = new Translation3d(0, 0, 0);
    if (ShootLevel == 2) {
      ShootingSpot = Constants.MIDDLE_SPOTS.get(Closest);
    } else {
      ShootingSpot = Constants.HIGH_SPOTS.get(Closest);
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
}