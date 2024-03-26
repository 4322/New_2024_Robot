package frc.robot.shooting;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.interpolation.GenericCalculator;
import frc.robot.util.interpolation.GenericFiringSolutionManager;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FiringSolutionManager implements GenericFiringSolutionManager<FiringSolution> {
  private final ArrayList<FiringSolution> solutions;
  private final GenericCalculator<FiringSolution> calculator;
  private final ObjectMapper objectMapper = new ObjectMapper();

  public FiringSolutionManager(ArrayList<FiringSolution> solutionArrayList, GenericCalculator<FiringSolution> calculator) {
    solutions = solutionArrayList;
    this.calculator = calculator;
    calculator.init(solutions);
  }

  public void addSolution(FiringSolution solution) {
    solutions.add(solution);
    calculator.whenAdded();
  }

  public void writeSolution(FiringSolution solution) {
    try {
      addSolution(solution);
      objectMapper.writeValue(new File("/media/sda1/FiringSolutions.json"), solutions);
      DriverStation.reportWarning("Wrote new solution to firing solution json", false);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to write new firing solution", false);
    }
  }

  public void loadSolutions() {
    List<FiringSolution> solutionList;
    try {
      solutionList =
          objectMapper.readValue(
              new File(Filesystem.getDeployDirectory().getPath() + "/FiringSolutions.json"),
              new TypeReference<ArrayList<FiringSolution>>() {});
      for (FiringSolution solution : solutionList) {
        addSolution(solution);
      }
      DriverStation.reportWarning("Loaded all firing solutions", false);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to load firing solutions", e.getStackTrace());
    }
  }

  public FiringSolution calcSolution(double currentMag, double currentDeg) {
    FiringSolution inputsToFind = new FiringSolution(currentMag, currentDeg);
    ArrayList<FiringSolution> selectedSolutions = calculator.find(inputsToFind);
    for (int i = 0; i < selectedSolutions.size(); i++) {
      Logger.recordOutput(
          "FiringSolutions/PointOfInterpolation/Solution " + i,
          selectedSolutions.get(i).toString());
    }
    ArrayList<Double> calculatedComponents =
        calculator.calculate(currentMag, currentDeg, selectedSolutions);
    return new FiringSolution(currentMag, currentDeg, calculatedComponents);
  }
}
