package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

// Our modified version of PathPlanner's PathFollowing Command
public class PathFollower extends CommandBase {
  private final Timer timer = new Timer();
  private final boolean usePID;
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final RamseteController controller;
  private final SimpleMotorFeedforward feedforward;
  private final DifferentialDriveKinematics kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
  private final PIDController leftController;
  private final PIDController rightController;
  private final BiConsumer<Double, Double> output;
  private final boolean useAllianceColor;

  private DifferentialDriveWheelSpeeds prevSpeeds;
  private double prevTime;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<ChassisSpeeds> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError = PathFollower::defaultLogError;

  public PathFollower(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.controller = controller;
    this.feedforward = feedforward;
    this.kinematics = kinematics;
    this.speedsSupplier = speedsSupplier;
    this.leftController = leftController;
    this.rightController = rightController;
    this.output = outputVolts;
    this.useAllianceColor = useAllianceColor;

    this.usePID = true;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  public PathFollower(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        controller,
        feedforward,
        kinematics,
        speedsSupplier,
        leftController,
        rightController,
        outputVolts,
        false,
        requirements);
  }

  public PathFollower(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      RamseteController controller,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.controller = controller;
    this.kinematics = kinematics;
    this.output = outputMetersPerSecond;

    this.feedforward = null;
    this.speedsSupplier = null;
    this.leftController = null;
    this.rightController = null;
    this.useAllianceColor = useAllianceColor;

    this.usePID = false;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  public PathFollower(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      RamseteController controller,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        controller,
        kinematics,
        outputMetersPerSecond,
        false,
        requirements);
  }

  @Override
  public void initialize() {
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    this.prevTime = -1;

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    PathPlannerTrajectory.PathPlannerState initialState = transformedTrajectory.getInitialState();

    this.prevSpeeds =
        this.kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

    this.timer.reset();
    this.timer.start();

    if (this.usePID) {
      this.leftController.reset();
      this.rightController.reset();
    }

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    double dt = currentTime - this.prevTime;

    if (this.prevTime < 0) {
      this.prevTime = currentTime;
      return;
    }

    Pose2d currentPose = this.poseSupplier.get();
    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);

    SmartDashboard.putNumberArray("Current Pose", new double[] {
      currentPose.getX(), 
      currentPose.getY(), 
      currentPose.getRotation().getDegrees()});

    SmartDashboard.putNumberArray("Desired Pose", new double[] {
      desiredState.poseMeters.getX(), 
      desiredState.poseMeters.getY(), 
      desiredState.poseMeters.getRotation().getDegrees()});
    
    System.out.println("\nCurrent Pose: ");
    System.out.print(currentPose);
    System.out.println("\nDesired State: ");
    System.out.print(desiredState);
    

    PathPlannerServer.sendPathFollowingData(desiredState.poseMeters, currentPose);

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
    DifferentialDriveWheelSpeeds targetWheelSpeeds =
        this.kinematics.toWheelSpeeds(targetChassisSpeeds);

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    System.out.println("Left Output: " + leftSpeedSetpoint);
    System.out.println("Right Output: " + rightSpeedSetpoint);

    SmartDashboard.putNumberArray("Wheel Speed Setpoints", new double[] {
      leftSpeedSetpoint,
      rightSpeedSetpoint
    });

    double leftOutput;
    double rightOutput;

    if (this.usePID) {
      double leftFeedforward =
          this.feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - this.prevSpeeds.leftMetersPerSecond) / dt);
      double rightFeedforward =
          this.feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - this.prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + this.leftController.calculate(
                  this.speedsSupplier.get().leftMetersPerSecond, leftSpeedSetpoint);
      rightOutput =
          rightFeedforward
              + this.rightController.calculate(
                  this.speedsSupplier.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    System.out.println("Left Output: " + leftOutput);
    System.out.println("Right Output: " + rightOutput);

    SmartDashboard.putNumberArray("Outputs", new double[] {
      leftOutput,
      rightOutput
    });

    this.output.accept(leftOutput, rightOutput);
    this.prevSpeeds = targetWheelSpeeds;
    this.prevTime = currentTime;

    if (logTargetPose != null) {
      logTargetPose.accept(desiredState.poseMeters);
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.poseMeters.getRotation()));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {
      this.output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PathFollower/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PathFollower/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber("PathFollower/rotationErrorDegrees", rotationError.getDegrees());
  }

  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<ChassisSpeeds> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    PathFollower.logActiveTrajectory = logActiveTrajectory;
    PathFollower.logTargetPose = logTargetPose;
    PathFollower.logSetpoint = logSetpoint;
    PathFollower.logError = logError;
  }
}
