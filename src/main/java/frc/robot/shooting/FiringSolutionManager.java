package frc.robot.shooting;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.interpolation.GenericCalculator;
import frc.robot.util.interpolation.GenericFiringSolutionManager;
import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class FiringSolutionManager implements GenericFiringSolutionManager<FiringSolution> {
  private final ArrayList<FiringSolution> solutions;
  private final GenericCalculator<FiringSolution> calculator;

  public static FiringSolutionManager createNew(GenericCalculator<FiringSolution> calculator) {
    return fromArrayList(new ArrayList<>(), calculator);
  }

  public static FiringSolutionManager fromJson(
      GenericCalculator<FiringSolution> calculator, String filepath) {
    return fromArrayList(readSolutions(filepath), calculator);
  }

  public static FiringSolutionManager fromArrayList(
      ArrayList<FiringSolution> solutionArrayList, GenericCalculator<FiringSolution> calculator) {
    final FiringSolutionManager manager = new FiringSolutionManager(solutionArrayList, calculator);
    manager.init();
    return manager;
  }

  private FiringSolutionManager(
      ArrayList<FiringSolution> solutionArrayList, GenericCalculator<FiringSolution> calculator) {
    solutions = solutionArrayList;
    this.calculator = calculator;
  }

  private void init() {
    calculator.init(solutions);
  }

  private static ArrayList<FiringSolution> readSolutions(String filepath) {
    // https://stackoverflow.com/questions/43981487/how-to-append-object-to-existing-json-file-with-jackson
    ArrayList<FiringSolution> solutionList = new ArrayList<>();
    final ObjectMapper mapper = new ObjectMapper();
    try {
      solutionList =
          mapper.readValue(new File(filepath), new TypeReference<ArrayList<FiringSolution>>() {});
      DriverStation.reportWarning("Loaded all firing solutions", false);
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to load firing solutions", e.getStackTrace());
    }
    return solutionList;
  }

  public static void writeSolution(FiringSolution solution, String filepath) {
    final File file = new File(filepath);
    final ObjectMapper mapper = new ObjectMapper();
    try {
      JsonGenerator g = mapper.getFactory().createGenerator(new FileOutputStream(file));
      mapper.writeValue(new File("/media/sda1/FiringSolutions.json"), solution);
      DriverStation.reportWarning("Wrote new solution to firing solution json", false);
      g.close();
    } catch (Exception e) {
      e.printStackTrace();
      DriverStation.reportError("Failed to write new firing solution", false);
    }
  }

  public void addSolution(FiringSolution solution) {
    solutions.add(solution);
    calculator.whenAdded();
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
