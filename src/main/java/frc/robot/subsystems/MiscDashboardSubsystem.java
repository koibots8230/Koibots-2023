package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MiscDashboardSubsystem extends SubsystemBase {

    public MiscDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        Shuffleboard.selectTab("SmartDashboard");
        SmartDashboard.putBoolean("Battery Alert", false);
        SmartDashboard.putNumber("Battery Voltage", getBatteryVoltage());

        /*
         * | My failed attempt at directly editing network tables
         * NetworkTableInstance inst = NetworkTableInstance.getDefault();
         * NetworkTable table = inst.getTable("datatable");
         * NetworkTableValue bool_false;
         * NetworkTableValue bool_true;
         * 
         * bool_false.makeBoolean(false);
         * bool_true.makeBoolean(true);
         * 
         * table.putValue("test_dashboard/Battery Alert", bool_true);
         */
    }

    @Override
    public void simulationPeriodic() {
        Shuffleboard.selectTab("SmartDashboard");
        SmartDashboard.putBoolean("Battery Alert", false);
        SmartDashboard.putNumber("Battery Voltage", getBatteryVoltage());

        /*
         * | My failed attempt at directly editing network tables
         * NetworkTableInstance inst = NetworkTableInstance.getDefault();
         * NetworkTable table = inst.getTable("datatable");
         * NetworkTableValue bool_false;
         * NetworkTableValue bool_true;
         * 
         * bool_false.makeBoolean(false);
         * bool_true.makeBoolean(true);
         * 
         * table.putValue("test_dashboard/Battery Alert", bool_true);
         */
    }

    public static double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static boolean getBatteryVoltageAlert() {
        if (RobotController.getBatteryVoltage() >= 12) {
            return true;
        }

        return false;

    }

}
