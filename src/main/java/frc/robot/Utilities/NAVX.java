package frc.robot.Utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class NAVX extends AHRS{
    private static NAVX gyro = new NAVX();

    NAVX() {
        super(SPI.Port.kMXP);
    }

    public static NAVX get() {
        return gyro;
    }
}
