package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

public class MiscDashboardSubsystem extends SubsystemBase {

    private AHRS gyro = new AHRS(Port.kMXP);
    // private int periodic_loop_counter = 0;
    private boolean voltage_alert = true;
    private double voltage = 11; // Voltage of the battery
    private double batteryCurrent = 0; // Current of the batter
    private double inSpeed = 10; // Speed of the intake motor
    private double inCurrent = 0; // Current of the intake motor
    public boolean is_cube = false; // Boolean to determine whether retreived object is a cube or cone
    private double midCurrent = 0; // Current of the midtake motor
    private double midSpeed = 10; // Speed of the midtake motor
    
    public MiscDashboardSubsystem(IntakeSubsystem intake, ShooterSubsystem shooter, TankDriveSubsystem drive) {
        ShuffleboardTab main_tab = Shuffleboard.getTab("Main");
        
        // Battery info:
        main_tab.addBoolean("Voltage Alert", () -> voltage_alert).withSize(1, 1).withPosition(2, 0);

        main_tab.addNumber("Main Battery Voltage", () -> voltage).withPosition(0, 0).
        withSize(2, 1).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", 10, "max", 14));

        main_tab.addNumber("Main Battery Current (A)", () -> batteryCurrent).withPosition(3, 0).
        withSize(2,2).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 600));

        main_tab.addNumber("Shooter Motor Speed (RPM)", () -> shooter.getShooterSpeed()).withPosition(0, 3).
        withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -5000, "max", 5000));

        main_tab.addNumber("Left Drive Motor Speed (RPM)", () -> drive.getLeftDriveSpeed()*(68/30)).withPosition(0, 1).
        withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -7500, "max", 7500));

        main_tab.addNumber("Right Drive Motor Speed (RPM)", () -> drive.getRightDriveSpeed()*(68/30)).withPosition(0, 2).
        withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -7500, "max", 7500));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        voltage_alert = getBatteryVoltageAlert();
        voltage = getBatteryVoltage();
    }

    @Override
    public void simulationPeriodic() {
        voltage_alert = getBatteryVoltageAlert();
        voltage = getBatteryVoltage();
        batteryCurrent = getBatteryCurrent();
    }

    public static double getBatteryVoltage() {
        return RobotController.getInputVoltage();
    }

    public static boolean getBatteryVoltageAlert() {
        if (RobotController.getInputVoltage() > 11) {
            return true;
        }
        return false;

    }

    public static double getBatteryCurrent() {
        return RobotController.getInputCurrent();
    }

    public static double checkDelay(int delay) {
        return (delay * ( 1 / 0.020 ));
    }

}