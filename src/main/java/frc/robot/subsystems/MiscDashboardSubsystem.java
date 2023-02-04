package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MiscDashboardSubsystem extends SubsystemBase {
    
    public MiscDashboardSubsystem() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Battery Alert", false);
        SmartDashboard.putNumber("Battery Voltage", getBatteryVoltage());
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("Battery Alert", false);
        SmartDashboard.putNumber("Battery Voltage", getBatteryVoltage());
    }

    public static double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static boolean getBatteryVoltageAlert() {
        if(RobotController.getBatteryVoltage() >= 12) {
            return true;
        }

        return false;

    }

}
