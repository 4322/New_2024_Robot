package frc.robot.util.interpolation;

public interface GenericFiringSolutionManager<S extends GenericFiringSolution> {
  public S calcSolution(double currentMag, double currentDeg);
}
