package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

public class MiscDashboardSubsystem extends SubsystemBase {

    // private int periodic_loop_counter = 0;
    private boolean voltage_alert = true;
    private double voltage = 11;
    private boolean current_alert = true;
    private double current = 0;
    public boolean is_cube = true;
    private String is_cube_text = "Cube";
    
    public MiscDashboardSubsystem() {
        ShuffleboardTab main_tab = Shuffleboard.getTab("Main");
        
        main_tab.addBoolean("Voltage Alert", () -> voltage_alert).withPosition(2, 0);

        main_tab.addNumber("Voltage", () -> voltage).withPosition(0, 0).
        withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("min", 10, "max", 14));

        main_tab.addNumber("Current", () -> current).withPosition(4, 0).
        withWidget(BuiltInWidgets.kDial);

        main_tab.addString("Cube or Cone", () -> is_cube_text).withPosition(0, 1)
        .withWidget(BuiltInWidgets.kTextView);
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

        if (is_cube) {
            is_cube_text = "Cube";
        } else {
            is_cube_text = "Cone";
        }
    
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

        if (is_cube) {
            is_cube_text = "Cube";
        } else {
            is_cube_text = "Cone";
        }
    
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
