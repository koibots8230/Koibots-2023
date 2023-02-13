package frc.robot.subsystems;

import java.util.Optional;

import org.ejml.equation.Variable;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public static Pose3d Bot3d = null;
  public static boolean VariablesDefined = false;
  public static Translation3d Closest = new Translation3d(0, 0, 0);
  public static double ClosestDistance = 0;
  private int count = 0;
  private int ShootLevel = 2;

  @Override
  public void periodic() {
    if (count == 10) {
      count = 0;
      final Optional<EstimatedRobotPose> BotThing = VisionSubsystem.photonPoseEstimator.update();
      if (BotThing.isPresent()) {
        Bot3d = BotThing.get().estimatedPose;
        VariablesDefined = true;
        double xDistance = 0;
        double yDistance = 0;
        double distance = 0;
        Translation2d Spot = new Translation2d(0, 0);
        for (int a = 0; a < 6; a++) {
          if (ShootLevel == 2) {
            Spot = Constants.MIDDLE_SPOTS.get(a).toTranslation2d();
          } else {
            Spot = Constants.HIGH_SPOTS.get(a).toTranslation2d();
          }
          xDistance = Bot3d.getX() - Spot.getX();
          yDistance = Bot3d.getY() - Spot.getY();
          distance = Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
          if (distance < ClosestDistance) {
            ClosestDistance = distance;
            Closest = Constants.MIDDLE_SPOTS.get(a);
          }
        }
      } else {
        VariablesDefined = false;
      }
    } else {
      count ++;
    }
  }

  @Override
  public void simulationPeriodic() {

  }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
}