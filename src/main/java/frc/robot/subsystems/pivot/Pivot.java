package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OrangeMath;

public class Pivot extends SubsystemBase {
  private double pivotTarget;
  private boolean pivotInitialized;
  private boolean isInCoast;

  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  //TODO: Need to set constants for pivot. 
  public Pivot(PivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        io.configurePID(Constants.Pivot.kP, Constants.Pivot.kI, Constants.Pivot.kD);
        break;
      case SIM:
        io.configurePID(Constants.Pivot.kP, Constants.Pivot.kI, Constants.Pivot.kD);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    if (Constants.Enabled.pivot && io.pivotIsInitialized()) {
      io.setPosition(inputs.targetPivotPosition);
      pivotTarget = inputs.targetPivotPosition;
    }
  }

  public void resetPivot() {
    io.setPosition(Constants.Pivot.defaultPivotPositionRotations);
    pivotTarget = Constants.Pivot.defaultPivotPositionRotations;
  }

  public void stopPivot() {
    if (Constants.Enabled.pivot && pivotInitialized) {
      io.stop();
    }
  }

  public void setPivotCoastMode() {
    if (Constants.Enabled.pivot) {
      isInCoast = true;
      io.setCoastMode();
    }
  }

  public void setPivotBrakeMode() {
    if (Constants.Enabled.pivot) {
      isInCoast = false;
      io.setBrakeMode();
    }
  }

  public boolean pivotIsAtPosition() {
    return OrangeMath.equalToEpsilon(
        inputs.pivotRotations, pivotTarget, Constants.Pivot.toleranceRotations);
  }

  public boolean pivotIsInitialized() {
    return pivotInitialized;
  }

  public boolean safeToPivot() {
    return (inputs.pivotRotations > Constants.Pivot.reverseSoftLimitThresholdRotations
        && inputs.pivotRotations < Constants.Pivot.forwardSoftLimitThresholdRotations);
  }

  public boolean pivotInCoast() {
    return isInCoast;
  }
}
