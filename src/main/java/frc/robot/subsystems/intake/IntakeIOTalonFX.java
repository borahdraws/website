// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeTalon;
  private final VoltageOut request;

  public IntakeIOTalonFX() {
    intakeTalon = new TalonFX(5);
    request = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePositionRadians = intakeTalon.getPosition().getValue().in(Radians);
    inputs.intakeVelocityRadiansPerSeconds =
        intakeTalon.getVelocity().getValue().in(RadiansPerSecond);
    inputs.intakeAppliedVoltage = intakeTalon.getMotorVoltage().getValueAsDouble();
    inputs.intakeCurrentAmperage = intakeTalon.getSupplyCurrent().getValue().in(Amps);
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    request.Output = voltage;
    intakeTalon.setControl(request);
  }
}
