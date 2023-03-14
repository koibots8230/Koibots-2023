package frc.robot.Utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class NAVX extends AHRS{
    private static NAVX gyro = new NAVX();

    NAVX() {
        super(SPI.Port.kMXP);

        if (DriverStation.getAlliance() == Alliance.Red) {
            this.setAngleAdjustment(180);
        }
    }

    public static NAVX get() {
        return gyro;
    }
}
