package frc.robot;

import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LoadCube;
import frc.robot.subsystems.IntakePositionSubsystem;

public class Constants {
  // =============================== In-Use Values =============================== \\

  // Constants for auto
  // DO. NOT. CHANGE. THESE. UNLESS AUTO IS ACTIVLEY BEING TESTED AND YOU KNOW
  // WHAT YOU ARE DOING.


  private static final PathConstraints AUTO_CONSTRAINTS = new PathConstraints(0, 0);

  public static final Hashtable<String, PathPlannerTrajectory> Paths = new Hashtable<String, PathPlannerTrajectory>() 
  {{

    put("Blue: Left Side - 3 Pieces", PathPlanner.loadPath("B_Left", AUTO_CONSTRAINTS));
    put("Blue: Left Side - 2 Pieces", PathPlanner.loadPath("B_Left_Score2", AUTO_CONSTRAINTS));
    put("Blue: Right Side - 3 Pieces", PathPlanner.loadPath("B_Right", AUTO_CONSTRAINTS));
    put("Blue: Right Side - 2 Pieces", PathPlanner.loadPath("B_Right_Score2", AUTO_CONSTRAINTS));
    put("Blue: Center - Coopertition", PathPlanner.loadPath("B_Center", AUTO_CONSTRAINTS));
    put("Blue: Center - Left piece first", PathPlanner.loadPath("B_Center_Top_First", AUTO_CONSTRAINTS));
    put("Blue: Center - Right piece first", PathPlanner.loadPath("B_Center_Bottom_First", AUTO_CONSTRAINTS));

    put("Red: Left Side - 3 Pieces", PathPlanner.loadPath("R_Left", AUTO_CONSTRAINTS));
    put("Red: Left Side - 2 Pieces", PathPlanner.loadPath("R_Left_Score2", AUTO_CONSTRAINTS));
    put("Red: Right Side - 3 Pieces", PathPlanner.loadPath("R_Right", AUTO_CONSTRAINTS));
    put("Red: Right Side - 2 Pieces", PathPlanner.loadPath("R_Right_Score2", AUTO_CONSTRAINTS));
    put("Red: Center - Coopertition", PathPlanner.loadPath("R_Center", AUTO_CONSTRAINTS));
    put("Red: Center - Left piece first", PathPlanner.loadPath("R_Center_Bottom_First", AUTO_CONSTRAINTS));
    put("Red: Center - Right piece first", PathPlanner.loadPath("R_Center_Top_First", AUTO_CONSTRAINTS));
    
   }};

  public static final Hashtable<String, Command> Events = new Hashtable<String, Command>() // TODO: Add Events to Paths
  {{
    put("Lower Intake", IntakePositionSubsystem.get().new FlipIntake());
    put("Pick Up Cube", new LoadCube());
    put("Auto-Balance", new AutoBalanceCommand());
  }};

  public static final double PAUSE_LENGTH = 0.3;
  public static final double CURRENT_CAP = 60;
  public static final double FLIP_INTAKE_DISTANCE = 2;

  public static final double DRIVE_ROTATIONS_TO_DISTANCE = 1; // TODO: Find this
  public static final double DRIVE_SPEED_COEFFICIENT = 0.85; 

  public static final double ROBOT_WIDTH_m = 0.6; // TODO: Double check this

  public static final double AUTOBALANCE_MOVE_LIMIT = 85;
  public static final double SHOOT_MOVE_LIMIT = 140;

  public static final double SHOOT_RIGHT_SPEED = 0.25;
  public static final double SHOOT_LEFT_SPEED = 0.25;

  public static final double AUTO_RIGHT_SPEED = 0.3;
  public static final double AUTO_LEFT_SPEED = 0.3;

  public static final double SHOOT_SECONDS = 0.9;

  // Slowmotion, deadzone, etc:
  public static final double SLOW_MODE_FACTOR = 0.5;
  public static final double AUTO_SPEED = 0.07;
  public static final double MAX_DRIVETRAIN_SPEED = .88;

  // Intake/Midtake Constants
  public static final double STARS_RUNNING_SPEED = 0.30;
  public static final double BELT_RUNNING_SPEED = 0.30;
  public static final double INTAKE_RUNNING_SPEED = 0.43;

  // Teleop Shooti
  public static final double COMMUNITY_SHOOTER_SPEED = .95;
  public static final double L2_SHOOTER_SPEED = .7;
  public static final double L1_SHOOTER_SPEED = .55;

  public static final double DEADZONE = 0.15; // Probably don't change this

  // Hardware Ports
  public static final int LEFT_DRIVE_MOTOR_1 = 15;
  public static final int LEFT_DRIVE_MOTOR_2 = 14;
  public static final int RIGHT_DRIVE_MOTOR_1 = 12;
  public static final int RIGHT_DRIVE_MOTOR_2 = 13;

  public static final int INTAKE_MOTOR = 10;
  public static final int RAISE_INTAKE_MOTOR = 11;

  public static final int SHOOTER_MOTOR_L = 4;
  public static final int SHOOTER_MOTOR_R = 5;

  public static final int MIDTAKE_MOTOR = 7;
  public static final int STAR_WHEELS_MOTOR_L = 8;
  public static final int STAR_WHEELS_MOTOR_R = 9;

  // ===============================Unused Values===============================

  public static final double RAISE_SPEED = 0.8;
  public static final double CURRENT_ZONE_AMPS = 1; // To be changed when we have an actual intake
  public static final double INTAKE_UP_POSITION = 1; // To be changed when we have an actual intake
  public static final double INTAKE_DOWN_POSITION = -1; // To be changed when we have an actual intake
  public static final double INTAKE_CHANGE_POSITION = 20;
  public static final double SENSOR_TRIGGERED = 2.5;
  public static final int BEAM_BREAK = 1;

  // for LED system
  public static final int LED_STRIP_LENGTH = 60;// the number of LEDs on each of the LED strips.
  // public static final int LEDPort1=0;
  // public static final int LEDPort2=0;//neither of these are known currently,
  // but they can be uncommented once we know the port numbers.

  // Auto Shooting
  public static final double SHOOTER_ANGLE = 45; // To be changed
  public static final double GRAVITY = 9.8; // If you can figure out a way to change this one, that's impressive
  public static final double SHOOTER_FROM_GROUND = 1; // To be changed
  public static final double VELOCITY_TO_SPEED = 0; // To be changed
  public static final double MIDDLE_HEIGHT = 23.5 * 0.0254;
  public static final double MIDDLE_X = 24.25 * 0.0254;

  public static final double Y1 = 43.125 * 0.0254;
  public static final double Y2 = 108.5 * 0.0254;
  public static final double Y3 = 174.625 * 0.0254;

  public static final Translation3d MC1 = new Translation3d(MIDDLE_X, Y1, MIDDLE_HEIGHT);
  public static final Translation3d MC2 = new Translation3d(MIDDLE_X, Y2, MIDDLE_HEIGHT);
  public static final Translation3d MC3 = new Translation3d(MIDDLE_X, Y3, MIDDLE_HEIGHT);

  public static List<Translation3d> MIDDLE_SPOTS = Arrays.asList(MC1, MC2, MC3);

  public static final double HIGH_HEIGHT = 35.5 * 0.0254;
  public static final double HIGH_X = 12.25 * 0.0254;

  public static final Translation3d HC1 = new Translation3d(HIGH_X, Y1, HIGH_HEIGHT);
  public static final Translation3d HC2 = new Translation3d(HIGH_X, Y2, HIGH_HEIGHT);
  public static final Translation3d HC3 = new Translation3d(HIGH_X, Y3, HIGH_HEIGHT);

  public static List<Translation3d> HIGH_SPOTS = Arrays.asList(HC1, HC2, HC3);

  public static final double MAX_SHOOTER_RANGE = 2;

  // Encoder Values
  public static final double RIGHT_ENCODER_ROTATIONS_TO_DISTANCE = 0;
  public static final double LEFT_ENCODER_ROTATIONS_TO_DISTANCE = 0;

  // Path Folowing Values TO BE CHANGED
  public static final double ks_VOLTS = 0;
  public static final double kv_VOLT_SECONDS_PER_METER = 0;
  public static final double ka_VOLT_SECONDS_SQUARED_PERMETER = 0;
  public static final double kp_DRIVE_VEL = 0;
  public static final double MAX_SPEED_METERS_PER_SECOND = 0;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;

  // public static final double TRACK_WIDTH_METERS = 0;
  // public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new
  // DifferentialDriveKinematics(
  // TRACK_WIDTH_METERS);

}