package frc.robot;

public class Constants {

// ====================================== Auto ====================================== \\
// Don't touch these unless you are activley tuning them

  public static final double AUTO_L2_SHOOTER_SPEED = 0.38;
  public static final double STATION_TO_HYBRID_SHOOTER_SPEED = 0.6;
  public static final double AUTO_SPEED = 0.07;

// ====================================== Teleop / Driver ====================================== \\

  // Drivetrain
  public static final double MAX_DRIVETRAIN_SPEED = .93;
  public static final double DRIVE_SPEED_COEFFICIENT = 0.90; 
  public static final double DRIVE_ROTATIONS_TO_DISTANCE = 0.04873967373;

  // Intake/Midtake Constants
  public static final double BELT_RUNNING_SPEED = 0.85;
  public static final double BELT_REVERSE_SPEED = -0.6;
  public static final double INTAKE_RUNNING_SPEED = 0.88;
  public static final double INTAKE_REVERSE_SPEED = -0.5;
  public static final double RAISE_SPEED = 0.25;

  // Teleop Shooting
  public static final double COMMUNITY_SHOOTER_SPEED = .95;
  public static final double L2_SHOOTER_SPEED = .43;
  public static final double L1_SHOOTER_SPEED = .28;
  public static final double HYBRID_SHOOTER_SPEED = .17;

  
  // Controller
  public static final double THUMBSTICK_DEADZONE = 0.15;
  public static final double TRIGGER_DEADZONE = 0.5;

  // Sensors
  public static final double SENSOR_TRIGGERED = .225; // Voltage at which a digital signal is considered activated

  public static final double CURRENT_CAP = 60;
  public static final double FLIP_INTAKE_DISTANCE = 5;
  

// ====================================== Hardware Ports ====================================== \\

  // Shooter
  public static final int SHOOTER_MOTOR_L = 5;
  public static final int SHOOTER_MOTOR_R = 8;

  // Indexer
  public static final int MIDTAKE_MOTOR = 4;
  public static final int BEAM_BREAK = 1; // Analog Input
  
  // Intake
  public static final int INTAKE_MOTOR = 10;

  // IntakePosition
  public static final int RAISE_INTAKE_MOTOR = 7;

  // TankDrive
  public static final int RIGHT_DRIVE_MOTOR_1 = 12;
  public static final int RIGHT_DRIVE_MOTOR_2 = 13;
  public static final int LEFT_DRIVE_MOTOR_1 = 15;
  public static final int LEFT_DRIVE_MOTOR_2 = 14;
}