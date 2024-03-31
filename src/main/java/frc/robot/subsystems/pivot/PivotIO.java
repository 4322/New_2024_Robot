package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double pivotRotations = 0.0;
    public double pivotRotationsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotStatorCurrentAmps = 0.0;
    public double pivotTempC = 0.0;

    public double pivotEncoderRotations = 0.0;
    public double pivotEncoderRotationsPerSec = 0.0;
    public double targetPivotPosition;

    public double heliumAbsRotations = 0.0;
    public double heliumRelativeRotations = 0.0;

    public boolean pivotIsAlive = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setPosition(double position) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void setBrakeMode() {}

  public default void setCoastMode() {}

  public default boolean pivotIsInitialized() {
    return false;
  }

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
