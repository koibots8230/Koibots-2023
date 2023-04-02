package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities.NAVX;
import frc.robot.subsystems.TankDriveSubsystem;

public class FixAngle extends CommandBase{
    
    public FixAngle() {
        addRequirements(TankDriveSubsystem.get());
    }

    public void execute() {
        if (NAVX.get().getYaw() > 2.5) {
            TankDriveSubsystem.get().setMotor(0, 0.1);
        } else if (NAVX.get().getYaw() < -2.5) {
            TankDriveSubsystem.get().setMotor(0.1, 0);
        }
    }

    @Override
    public void end (boolean i) {
        TankDriveSubsystem.get().setMotor(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(NAVX.get().getYaw()) < 2.5;
    }
}
