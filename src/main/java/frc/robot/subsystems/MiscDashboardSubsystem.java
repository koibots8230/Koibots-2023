package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.Map;

public class MiscDashboardSubsystem extends SubsystemBase {

    // private int periodic_loop_counter = 0;
    private boolean voltage_alert = true;
    private double voltage = 11; // Voltage of the battery
    private double batteryCurrent = 0; // Current of the batter
    private double inSpeed = 10; // Speed of the intake motor
    private double inCurrent = 0; // Current of the intake motor
    public boolean is_cube = false; // Boolean to determine whether retreived object is a cube or cone
    private double midCurrent = 0; // Current of the midtake motor
    private double midSpeed = 10; // Speed of the midtake motor
    
    public MiscDashboardSubsystem(IntakeSubsystem intake, ShooterSubsystem shooter) {
        ShuffleboardTab main_tab = Shuffleboard.getTab("Main");
        
        // Battery info:
        main_tab.addBoolean("Voltage Alert", () -> voltage_alert).withSize(1, 1).withPosition(2, 0);

        main_tab.addNumber("Main Battery Voltage", () -> voltage).withPosition(0, 0).
        withSize(2, 1).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", 10, "max", 14));

        main_tab.addNumber("Main Battery Current (A)", () -> batteryCurrent).withPosition(3, 0).
        withSize(2,2).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 600));

        //The motor speeds:
        //main_tab.addNumber("Intake Motor Speed (RPM)", () -> inSpeed).withPosition(0, 2).
        //withSize(3,1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 600)); //Disclaimer: I DON'T KNOW THE ACTUAL TOP SPEED.

        //main_tab.addNumber("Midtake Motor Speed (RPM)", () -> midSpeed).withPosition(3, 2).
        //withSize(3,1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 600)); //Disclaimer: I DON'T KNOW THE ACTUAL TOP SPEED.

        main_tab.addNumber("Raise Intake Motor Current (A)", () -> intake.getIntakeRaiseMotor().getOutputCurrent()).withPosition(0, 3).
        withSize(3,1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 600));

        main_tab.addNumber("Shooter Motor Speed (RPM)", () -> shooter.getShooterSpeed()).withPosition(0, 4).
        withSize(3, 1).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", 0, "max", 600));
        
        // Cube or cone? Place your bets!!
        //main_tab.addBoolean("Cube or Cone", () -> is_cube).withPosition(0, 1).
        //withSize(3,1).withWidget(BuiltInWidgets.kBooleanBox).withProperties(Map.of("Color when true", "#880088", "Color when false", "#FFEE00"));
    }

    @Override
    public void periodic() {
        /* if (periodic_loop_counter >= checkDelay(2)) {
            // code
            periodic_loop_counter = 0;
        }
        periodic_loop_counter += 1; */

        voltage_alert = getBatteryVoltageAlert();
        voltage = getBatteryVoltage();
    }

    @Override
    public void simulationPeriodic() {
        /* if (periodic_loop_counter >= checkDelay(2)) {
            // code
            periodic_loop_counter = 0;
        }
        periodic_loop_counter += 1; */

        voltage_alert = getBatteryVoltageAlert();
        voltage = getBatteryVoltage();
        batteryCurrent = getBatteryCurrent();
    }

    public static double getBatteryVoltage() {
        return RobotController.getInputVoltage();
        // return RobotController.getBatteryVoltage();
    }

    public static boolean getBatteryVoltageAlert() {
        if (RobotController.getInputVoltage() >= 12) {
            return true;
        }

        return false;

        /* if (RobotController.getBatteryVoltage() >= 12) {
            return true;
        }

        return false; */

    }

    public static double getBatteryCurrent() {
        return RobotController.getInputCurrent();
    }

    public static double checkDelay(int delay) {
        return (delay * ( 1 / 0.020 ));
    }

}