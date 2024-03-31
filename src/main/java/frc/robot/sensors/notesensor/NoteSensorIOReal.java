package frc.robot.sensors.notesensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class NoteSensorIOReal implements NoteSensorIO {
  DigitalInput intakeInput;
  DigitalInput tunnelInput;

  public NoteSensorIOReal(int intakeSensorID, int tunnelSensorID) {
    intakeInput = new DigitalInput(intakeSensorID);
    tunnelInput = new DigitalInput(tunnelSensorID);
  }

  @Override
  public void updateInputs(NoteSensorIOInputs inputs) {
    // by default the beam returns true when not broken
    inputs.tunnelBeamBroken = !tunnelInput.get();
    inputs.intakeBeamBroken = !intakeInput.get();
  }
}
