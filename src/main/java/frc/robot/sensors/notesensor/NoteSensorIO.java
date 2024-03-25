package frc.robot.sensors.notesensor;

import org.littletonrobotics.junction.AutoLog;

public interface NoteSensorIO {
  @AutoLog
  public static class NoteSensorIOInputs {
    public boolean tunnelBeamBroken = true;
    public boolean intakeBeamBroken = true;
  }

  public default void updateInputs(NoteSensorIOInputs inputs) {}
}
