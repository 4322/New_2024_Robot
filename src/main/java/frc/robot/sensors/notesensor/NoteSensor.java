package frc.robot.sensors.notesensor;

public class NoteSensor {
  private final NoteSensorIO io;
  private final NoteSensorIOInputsAutoLogged inputs = new NoteSensorIOInputsAutoLogged();

  public NoteSensor(NoteSensorIO io) {
    this.io = io;
  }

  // as NoteSensor is not a subsystem, must be called periodically in RobotContainer
  public void periodic() {
    io.updateInputs(inputs);
  }

  public boolean isIntakeBeamBroken() {
    return inputs.intakeBeamBroken;
  }

  public boolean isTunnelBeamBroken() {
    return inputs.tunnelBeamBroken;
  } 
}
