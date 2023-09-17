package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
   private static final Indexer instance = new Indexer();
   private final IndexerIO io;
   private final IndexerIOStateAutoLogged state = new IndexerIOStateAutoLogged();
   public static Indexer get() {
      return instance;
   }

   private Indexer() {
      if (Robot.isReal()) {
         io = new IndexerIOSparkMax();
      } else {
         io = null; // TODO: Write sim version
      }
   }

   @Override
   public void periodic() {
      io.updateState(state);
      Logger.getInstance().processInputs("Indexer", state);
   }

   public void flipUseBeamBreak() {
      state.useBeamBreak = !state.useBeamBreak;
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

   public class RunUntilBeam extends CommandBase {
      double triggered;
      public RunUntilBeam() {
         addRequirements(Indexer.this);
      }

      @Override
      public void initialize() {
         Indexer.this.io.run();
         triggered = 0;
      }

      @Override
      public void execute() {
         if (Indexer.this.state.beamBreakVoltage < Constants.SENSOR_TRIGGERED) { //TODO: Is this meant to be a less than?
            triggered++;
         } else {
            triggered--;
         }
      }

      @Override
      public boolean isFinished() {
         return triggered >= 2 && Indexer.this.state.useBeamBreak;
      }

      @Override
      public void end(boolean interrupted) {
         Indexer.this.io.stop();
      }
   }
}
