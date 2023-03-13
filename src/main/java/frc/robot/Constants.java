package frc.robot;

import java.util.Hashtable;
import java.util.List;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities.TimedCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Constants {
  // DO. NOT. CHANGE. THESE. UNLESS YOU KNOW WHAT YOU ARE DOING

// ====================================== Auto ====================================== \\
  private static final PathConstraints AUTO_CONSTRAINTS = new PathConstraints(0, 0); // TODO: Find these

  public static final PIDConstants AUTO_PID = new PIDConstants(5, 0, 0); // TODO: Maybe tune these

  public static final Hashtable<String, List<PathPlannerTrajectory>> PATHS = new Hashtable<String, List<PathPlannerTrajectory>>() 
  {{

    put("Blue: Left Side - 3 Pieces", PathPlanner.loadPathGroup("B_Left", AUTO_CONSTRAINTS));
    put("Blue: Left Side - 2 Pieces", PathPlanner.loadPathGroup("B_Left_Score2", AUTO_CONSTRAINTS));
    put("Blue: Right Side - 3 Pieces", PathPlanner.loadPathGroup("B_Right", AUTO_CONSTRAINTS));
    put("Blue: Right Side - 2 Pieces", PathPlanner.loadPathGroup("B_Right_Score2", AUTO_CONSTRAINTS));
    put("Blue: Center - Coopertition", PathPlanner.loadPathGroup("B_Center", AUTO_CONSTRAINTS));
    put("Blue: Center - Left piece first", PathPlanner.loadPathGroup("B_Center_Top_First", AUTO_CONSTRAINTS));
    put("Blue: Center - Right piece first", PathPlanner.loadPathGroup("B_Center_Bottom_First", AUTO_CONSTRAINTS));

    put("Red: Left Side - 3 Pieces", PathPlanner.loadPathGroup("R_Left", AUTO_CONSTRAINTS));
    put("Red: Left Side - 2 Pieces", PathPlanner.loadPathGroup("R_Left_Score2", AUTO_CONSTRAINTS));
    put("Red: Right Side - 3 Pieces", PathPlanner.loadPathGroup("R_Right", AUTO_CONSTRAINTS));
    put("Red: Right Side - 2 Pieces", PathPlanner.loadPathGroup("R_Right_Score2", AUTO_CONSTRAINTS));
    put("Red: Center - Coopertition", PathPlanner.loadPathGroup("R_Center", AUTO_CONSTRAINTS));
    put("Red: Center - Left piece first", PathPlanner.loadPathGroup("R_Center_Bottom_First", AUTO_CONSTRAINTS));
    put("Red: Center - Right piece first", PathPlanner.loadPathGroup("R_Center_Top_First", AUTO_CONSTRAINTS));

  }};

  public static final Hashtable<String, Command> EVENTS = new Hashtable<String, Command>()
  {{

    put("Lower Intake", IntakePositionSubsystem.get().new FlipIntake());
    put("Pick Up Cube", new TimedCommand(new LoadCube(), (double) 1));
    put("Auto-Balance", new AutoBalanceCommand()); 
    put("Far Shot", new TimedCommand(ShooterSubsystem.get().new Shoot(FAR_SPEED), SHOOT_TIME));
    put("CS to L1", new TimedCommand(ShooterSubsystem.get().new Shoot(CS_TO_L1_SPEED), SHOOT_TIME));
    put("CS to Hybrid", new TimedCommand(ShooterSubsystem.get().new Shoot(CS_TO_HYBRID_SPEED), SHOOT_TIME));
    put("Ground to L1", new TimedCommand(ShooterSubsystem.get().new Shoot(GROUND_TO_L1_SPEED), SHOOT_TIME));
    put("Ground to Hybrid", new TimedCommand(ShooterSubsystem.get().new Shoot(GROUND_TO_HYBRID_SPEED), SHOOT_TIME));

  }};

  public static final SimpleMotorFeedforward PP_FEED_FORWARD = new SimpleMotorFeedforward(0.2, 0.4); // TODO: Tune these

  public static final double DRIVE_ROTATIONS_TO_DISTANCE = 1; // TODO: Find this

  public static final double ROBOT_WIDTH_m = 0.6; // TODO: Double check this

  public static final double AUTO_SPEED = 0.07;
  private static final double FAR_SPEED = 1; // TODO: unknown
  private static final double CS_TO_L1_SPEED = 0.5;// TODO: unknown
  private static final double CS_TO_HYBRID_SPEED = 0.5; // TODO: unknown
  private static final double GROUND_TO_L1_SPEED = 0.5; // TODO: unknown
  private static final double GROUND_TO_HYBRID_SPEED = 0.5; // TODO: unknown

  private static final double SHOOT_TIME = 0.5;

// ====================================== Teleop / Driver ====================================== \\

  public static final double MAX_DRIVETRAIN_SPEED = .88;
  // Intake/Midtake Constants
  public static final double STARS_RUNNING_SPEED = 0.30;
  public static final double BELT_RUNNING_SPEED = 0.30;
  public static final double INTAKE_RUNNING_SPEED = 0.43;

  public static final double DRIVE_SPEED_COEFFICIENT = 0.85; 

  public static final double RAISE_SPEED = 0.8;

  // Teleop Shooti
  public static final double COMMUNITY_SHOOTER_SPEED = .95;
  public static final double L2_SHOOTER_SPEED = .7;
  public static final double L1_SHOOTER_SPEED = .55;
  
  public static final double THUMBSTICK_DEADZONE = 0.15; // Probably don't change this
  public static final double TRIGGER_DEADZONE = -0.5;

  public static final double SENSOR_TRIGGERED = 2.5; // Voltage at which a digital signal is considered activated

  public static final double CURRENT_CAP = 60;
  public static final double FLIP_INTAKE_DISTANCE = 2;
  
  // for LED system
  public static final int LED_STRIP_LENGTH = 60; // the number of LEDs on each of the LED strips.

// ====================================== Hardware Ports ====================================== \\

  // new TimedCommand(ShooterSubsystem
  public static final int SHOOTER_MOTOR_L = 4;
  public static final int SHOOTER_MOTOR_R = 5;

  // IndexerSubsytem
  public static final int MIDTAKE_MOTOR = 7;
  public static final int STAR_WHEELS_MOTOR = 8;
  public static final int BEAM_BREAK = 1; // Analog Input
  
  // IntakeSubsystem
  public static final int INTAKE_MOTOR = 10;

  // IntakePositionSubsystem
  public static final int RAISE_INTAKE_MOTOR = 11;

  // TankDriveSubsystem
  public static final int RIGHT_DRIVE_MOTOR_1 = 12;
  public static final int RIGHT_DRIVE_MOTOR_2 = 13;
  public static final int LEFT_DRIVE_MOTOR_1 = 15;
  public static final int LEFT_DRIVE_MOTOR_2 = 14;

}