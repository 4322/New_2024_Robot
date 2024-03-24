// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.FlywheelIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  private final FlywheelIO topFlyWheelIO;
  private final FlywheelIO bottomFlyWheelIO;
  private final FlywheelIOInputsAutoLogged topFlywheelInputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged bottomFlywheelInputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  /** Creates a new Flywheel. */
  public Outtake(FlywheelIO topIO, FlywheelIO bottomIO) {
    topFlyWheelIO = topIO;
    bottomFlyWheelIO = bottomIO;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        topIO.configurePID(1.0, 0.0, 0.0);
        bottomIO.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        topIO.configurePID(0.5, 0.0, 0.0);
        bottomIO.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    topFlyWheelIO.updateInputs(topFlywheelInputs);
    bottomFlyWheelIO.updateInputs(bottomFlywheelInputs);
    Logger.processInputs("Top Flywheel", topFlywheelInputs);
    Logger.processInputs("Bottom Flywheel", bottomFlywheelInputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double topVolts, double bottomVolts) {
    topFlyWheelIO.setVoltage(topVolts);
    bottomFlyWheelIO.setVoltage(bottomVolts);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double topVelocityRPM, double bottomVelocityRPM) {
    var topVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(topVelocityRPM);
    var bottomVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(bottomVelocityRPM);

    topFlyWheelIO.setVelocity(topVelocityRadPerSec, ffModel.calculate(topVelocityRadPerSec));
    bottomFlyWheelIO.setVelocity(bottomVelocityRadPerSec, ffModel.calculate(bottomVelocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Top Flywheel/SetpointRPM", topVelocityRPM);
    Logger.recordOutput("Bottom Flywheel/SetpointRPM", bottomVelocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    topFlyWheelIO.stop();
    bottomFlyWheelIO.stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getTopVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(topFlywheelInputs.velocityRadPerSec);
  }

  @AutoLogOutput
  public double getBottomVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(bottomFlywheelInputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getTopCharacterizationVelocity() {
    return topFlywheelInputs.velocityRadPerSec;
  }

  public double getBottomCharacterizationVelocity() {
    return bottomFlywheelInputs.velocityRadPerSec;
  }
}
