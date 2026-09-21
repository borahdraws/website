// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
  DCMotorSim turretSimMotor =
    new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60(1), 
        0.005, 
        3.0
      ),
      DCMotor.getKrakenX60(1)
    );

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretSimMotor.update(0.02);
    
    inputs.turretAngleRadians = turretSimMotor.getAngularPositionRad();
    inputs.turretAppliedVoltage = turretSimMotor.getInputVoltage();
  }

  @Override
  public void setTurretVoltage(double volts) {
    turretSimMotor.setInputVoltage(volts);
  }
}
