// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  private double simElevatorAppliedVolts = 0.0;

  ElevatorSim elevatorSim =
    new ElevatorSim(
      DCMotor.getKrakenX60(2),
      40.0,
      Units.lbsToKilograms(15),
      Units.inchesToMeters(1),
      Units.inchesToMeters(0),
      Units.inchesToMeters(48),
      true,
      0
    );

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);

    inputs.elevatorLeaderConnected = true;
    inputs.elevatorFollowerConnected = true;

    inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = simElevatorAppliedVolts;
    
    // divided by number of motors (aka 2)
    inputs.elevatorLeaderCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
    inputs.elevatorFollowerCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
  }

  @Override
  public void setElevatorVoltage(double volts) {
    simElevatorAppliedVolts =
      MathUtil.clamp(
        volts,
        -12.0,
        12.0
      );

    elevatorSim.setInputVoltage(simElevatorAppliedVolts);
  }

  @Override
  public void resetElevatorPosition(double meters) {
    elevatorSim.setState(meters, 0.0);
  }

  @Override
  public void stopElevator() {
    setElevatorVoltage(0.0);
  }
}
