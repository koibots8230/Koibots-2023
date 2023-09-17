package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static final Intake instance = new Intake();
    private final IntakeIO io;
    private final IntakeIOStateAutoLogged state = new IntakeIOStateAutoLogged();
    private boolean intakeUp = true;

    public static Intake get() {
        return instance;
    }

    private Intake() {
        if (Robot.isReal()) {
            io = new IntakeIOSparkMax();
        } else {
            io = null; // TODO: Create sim version
        }
    }

    @Override
    public void periodic() {
        state.intakeUp = this.intakeUp;

        io.updateState(state);
        Logger.getInstance().processInputs("Intake", state);
    }

    public void clearPositionStickies() {
        io.clearPositionStickies();
    }

    public void setIntakePositionBrake() {
        io.setPositionBrake();
    }

    public void setIntakePositionCoast() {
        io.setPositionCoast();
    }

    public void run() {
        io.run();
    }

    public void runReversed() {
        io.runReversed();
    }

    public void stop() {
        io.stop();
    }

    public class IntakeUpDown extends CommandBase {
        final boolean up;
        final LinearFilter voltageFilter;

        public IntakeUpDown(boolean _up) {
            up = _up;
            voltageFilter = LinearFilter.movingAverage(5);
            addRequirements(Intake.get());
        }

        @Override
        public void initialize() {
            Intake.this.intakeUp = !Intake.this.intakeUp;
            if (up) {
                Intake.get().io.setIntakePositionSpeed(.3);
            } else {
                Intake.get().io.setIntakePositionSpeed(-.3);
            }
        }

        @Override
        public boolean isFinished() {
            return voltageFilter.calculate(Math.abs(Intake.get().state.intakePositionCurrent)) > Constants.CURRENT_CAP;
        }

        @Override
        public void end(boolean interrupted) {
            Intake.get().io.stopIntakePosition();
        }
    }
}
