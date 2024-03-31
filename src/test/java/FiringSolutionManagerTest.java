import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.shooting.FiringSolution;
import frc.robot.shooting.FiringSolutionManager;
import frc.robot.util.interpolation.Calculator1D;
import org.junit.jupiter.api.*;

public class FiringSolutionManagerTest {
  @Test
  public void testFromJson() {
    FiringSolutionManager manager =
        FiringSolutionManager.fromJson(
            new Calculator1D<>(), getClass().getResource("TestSolutions.json").getFile());

    FiringSolution calcSolution1 = manager.calcSolution(7.5, 0);
    assertEquals(calcSolution1.getShotMag(), 7.5);
    assertEquals(calcSolution1.getShotDeg(), 0);
    assertEquals(calcSolution1.getFlywheelSpeed(), 20);
    assertEquals(calcSolution1.getShotRotations(), 20);

    FiringSolution calcSolution2 = manager.calcSolution(15, 0);
    assertEquals(calcSolution2.getShotMag(), 15);
    assertEquals(calcSolution2.getShotDeg(), 0);
    assertEquals(calcSolution2.getFlywheelSpeed(), 35);
    assertEquals(calcSolution2.getShotRotations(), 35);

    FiringSolution calcSolution3 = manager.calcSolution(25, 0);
    assertEquals(calcSolution3.getShotMag(), 25);
    assertEquals(calcSolution3.getShotDeg(), 0);
    assertEquals(calcSolution3.getFlywheelSpeed(), 45);
    assertEquals(calcSolution3.getShotRotations(), 45);
  }
}
