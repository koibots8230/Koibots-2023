package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{

    final PhotonPoseEstimator photonPoseEstimator;
    final PhotonCamera camera;
    final Transform3d robotToCam;
    AprilTagFieldLayout aprilTagFieldLayout;
    SendableChooser<Boolean> m_sideChooser = new SendableChooser<>();

    public VisionSubsystem() {
        camera = new PhotonCamera("camera");
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        aprilTagFieldLayout = null;
        try {
            if (m_sideChooser.getSelected()) {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("../../../Deploy/BlueAprilTagLayout.json");
            } else {
                aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource("../../../Deploy/RedAprilTagLayout.json");
            }
            }
        catch (IOException ioexcept) {
            System.err.println("File did not exist! Try fixing your settings");  
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCam);
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
        if (pose.isPresent()) {
            EstimatedRobotPose real_pose = pose.get();
            SmartDashboard.putNumber("Estimated X", real_pose.estimatedPose.getX());
            SmartDashboard.putNumber("Estimated Y", real_pose.estimatedPose.getY());
        }
    }
}
