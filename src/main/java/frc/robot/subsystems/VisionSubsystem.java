package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{

    public static PhotonPoseEstimator photonPoseEstimator;
    final PhotonCamera camera;
    final Transform3d robotToCam;
    AprilTagFieldLayout aprilTagFieldLayout;
    private Boolean side = false;

    boolean m_sideChooser = true;
    public VisionSubsystem(Boolean _side) {
        camera = new PhotonCamera("KoiBotsCamera");
        robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        aprilTagFieldLayout = null;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            if (side == false) {
                aprilTagFieldLayout.setOrigin(new Pose3d(651.25*0.0254, 0.0, 0.0, new Rotation3d(0, 0, Math.PI)));
            }
        } catch (IOException ioexcept) {
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
