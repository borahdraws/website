// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {

  private double simVoltage = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAppliedVoltage = simVoltage;
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    this.simVoltage = voltage;
  }
}
