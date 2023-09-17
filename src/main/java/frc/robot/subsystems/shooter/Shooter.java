package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static final Shooter instance = new Shooter();
    private final ShooterIO io;
    private final ShooterIOStateAutoLogged state = new ShooterIOStateAutoLogged();

    public static Shooter get() {
        return instance;
    }

    private Shooter() {
        if (Robot.isReal()) {
            io = new ShooterIOSparkMax();
        } else {
            io = null; // TODO: Create sim version
        }
    }

    @Override
    public void periodic() {
        io.updateState(state);
        Logger.getInstance().processInputs("Shooter", state);
    }

    public void shoot(double speed) {
        io.setVoltages(speed, speed);
    }

    public void stop() {
        io.stop();
    }
}
