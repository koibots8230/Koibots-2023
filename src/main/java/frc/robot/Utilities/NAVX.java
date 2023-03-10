package frc.robot.Utilities;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TankDriveSubsystem;

public class NAVX extends AHRS{
    private static NAVX gyro = new NAVX();

    NAVX() {
        super(SPI.Port.kMXP);
    }

    public static NAVX getGyro() {
        return gyro;
    }
    
    public static class RelativeDrive extends CommandBase {
        double leftSpeed;
        double rightSpeed;
        double leftDistance;
        double rightDistance;
        double m_theta;
        TankDriveSubsystem m_drive;
        boolean turnNow = false;
        boolean end = false;
        double[] initialEncoderPositions;
        SparkMaxAbsoluteEncoder rightAbsoluteEncoder;
        SparkMaxAbsoluteEncoder leftAbsoluteEncoder;
        double halfRobotWidth = Constants.ROBOT_WIDTH_m / 2;
        double m_x;
        double m_y;

        /**
         * Create a new object to control a SPARK MAX motor Controller
         *
         * @param forwardsDistance_m        The distance (in meters) the robot drives forward or backwards. Must be positive
         * @param horizontalDistance_m        The distance (in meters) the robot drives left or right. Must be positive
         * @param omega    The gyro angle the robot should be at once the command has finished
         * @param reflectX Along with reflectY controlls the relative quadrant the bot ends up in
         * @param reflectY 
         */
        RelativeDrive(double forwardsDistance_m, double horizontalDistance_m, double theta, boolean reflectX, boolean reflectY, TankDriveSubsystem drive) {
            leftAbsoluteEncoder = drive.getLeftAbsoluteEncoder();
            rightAbsoluteEncoder = drive.getRightAbsoluteEncoder();

            leftDistance = ellipseCircumference(forwardsDistance_m + halfRobotWidth,  horizontalDistance_m + halfRobotWidth);
            rightDistance = ellipseCircumference(forwardsDistance_m - halfRobotWidth, horizontalDistance_m - halfRobotWidth);

            leftSpeed = leftDistance / rightDistance;
            rightSpeed = rightDistance / leftDistance;

            if (reflectY) {
                leftSpeed += rightSpeed;
                rightSpeed = leftSpeed - rightSpeed;
                leftSpeed -= rightSpeed;
            }
            if (reflectX) {
                leftSpeed *= -1;
                rightSpeed *= -1;
            }

            m_theta = theta;
            m_drive = drive;
            addRequirements(drive);
        }

        RelativeDrive(double forwardsDistance_m, double horizontalDistance_m, double omega, TankDriveSubsystem drive) {
            this(forwardsDistance_m, horizontalDistance_m, omega, false, false, drive);
        }

        @Override
        public void initialize() {
            NAVX.getGyro().resetDisplacement();
            m_drive.setMotor(leftSpeed, rightSpeed);
        }

        @Override
        public void execute() {
            if (
            (NAVX.getGyro().getDisplacementX() < m_x + Constants.ROBOT_WIDTH_m && NAVX.getGyro().getDisplacementX() > m_x) ||
            (NAVX.getGyro().getDisplacementY() < m_y + halfRobotWidth && NAVX.getGyro().getDisplacementY() > m_y - halfRobotWidth)) {
                turnNow = true;
                if (NAVX.getGyro().getYaw() < m_theta) {
                    m_drive.setMotor(Constants.TURN_SPEED, -Constants.TURN_SPEED);
                } else {
                    m_drive.setMotor(-Constants.TURN_SPEED, Constants.TURN_SPEED);
                }
            }
        }

        @Override
        public boolean isFinished() {
            if (turnNow && NAVX.getGyro().getYaw() < m_theta + 1 && NAVX.getGyro().getYaw() > m_theta -1) {
                return true;
            }
            return false;
        }

        public static double ellipseCircumference(double a, double b) {
            double c = Math.PI * (a + b);
            double d = 3 * Math.pow((a - b), 2);
            double e = Math.pow((a + b), 2);
            double f = (-3 * d) / e;
            double g = 4 + (10 / Math.sqrt(1 - f));
            return c * g;
        }
    }
}
