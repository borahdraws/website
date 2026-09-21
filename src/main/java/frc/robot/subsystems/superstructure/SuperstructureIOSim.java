// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class SuperstructureIOSim implements SuperstructureIO {

  private double simElevatorAppliedVolts = 0.0;
  private double simArmAppliedVolts = 0.0;

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

  SingleJointedArmSim armSim =
    new SingleJointedArmSim(DCMotor.getKrakenX60(1),
      25,
      SingleJointedArmSim.estimateMOI(
        Units.inchesToMeters(12),
        Units.lbsToKilograms(5)
      ),
      Units.inchesToMeters(12),
      Units.degreesToRadians(-30), 
      Units.degreesToRadians(120),
      true,
      Units.degreesToRadians(-30)
    );

  @Override
  public void updateInputs(SuperstructureIOInputs inputs) {
    elevatorSim.update(0.02);
    armSim.update(0.02);

    inputs.elevatorLeaderConnected = true;
    inputs.elevatorFollowerConnected = true;
    inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = simElevatorAppliedVolts;

    // divided by number of motors (aka 2)
    inputs.elevatorLeaderCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
    inputs.elevatorFollowerCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
    
    // (Fahrenheit - 32) * 5.0 / 9.0
    inputs.elevatorLeaderTempCelsius = (77 - 32) * 5.0 / 9.0;
    inputs.elevatorFollowerTempCelsius = (77 - 32) * 5.0 / 9.0;
 
    inputs.armConnected = true;
    inputs.armPositionRad = armSim.getAngleRads();
    inputs.armVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = simArmAppliedVolts;
    inputs.armCurrentAmps = armSim.getCurrentDrawAmps();

    // (Fahrenheit - 32) * 5.0 / 9.0
    inputs.armTempCelsius = (77 - 32) * 5.0 / 9.0;
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
  public void setArmVoltage(double volts) {
    simArmAppliedVolts = MathUtil.clamp(
      volts,
      -12.0,
      12.0
    );

    armSim.setInputVoltage(simArmAppliedVolts);
  }

  @Override
  public void stopSuperstructure() {
    setElevatorVoltage(0.0);
    setArmVoltage(0.0);
  }


}
