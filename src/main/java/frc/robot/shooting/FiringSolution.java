package frc.robot.shooting;

import frc.robot.util.interpolation.GenericFiringSolution;
import java.util.ArrayList;

public class FiringSolution extends GenericFiringSolution {
  private final double shotMag;
  private final double shotDeg; // angle from head-on to target (top-down to field)
  private final double flywheelSpeed;
  private final double shotRotations; // angle at which we should shoot (robot side view)

  // Needed for json library. Might crash without it
  private FiringSolution() {
    this.shotMag = 0;
    this.shotDeg = 0;
    this.flywheelSpeed = 0;
    this.shotRotations = 0;
  }

  public FiringSolution(
      double shotMag, double shotDeg, double flywheelSpeed, double shotRotations) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = flywheelSpeed;
    this.shotRotations = shotRotations;
  }

  protected FiringSolution(double shotMag, double shotDeg, ArrayList<Double> componentList) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = componentList.get(0);
    this.shotRotations = componentList.get(1);
  }

  protected FiringSolution(double shotMag, double shotDeg) {
    this.shotMag = shotMag;
    this.shotDeg = shotDeg;
    this.flywheelSpeed = 0;
    this.shotRotations = 0;
  }

  @Override
  protected ArrayList<Double> toComponentList() {
    ArrayList<Double> list = new ArrayList<>();
    list.add(flywheelSpeed);
    list.add(shotRotations);
    return list;
  }

  @Override
  public double getShotMag() {
    return shotMag;
  }

  @Override
  public double getShotDeg() {
    return shotDeg;
  }

  public double getFlywheelSpeed() {
    return flywheelSpeed;
  }

  public double getShotRotations() {
    return shotRotations;
  }

  @Override
  public String toString() {
    // used for logging purposes
    return "[ShotRotations: " + shotRotations + "], [FlywheelSpeed: " + flywheelSpeed + "]";
  }
}
