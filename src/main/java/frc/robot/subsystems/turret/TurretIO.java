// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretIO {

  @AutoLog
  public static class TurretIOInputs {
    public double turretAngleRadians = 0.0;
    public double turretAppliedVoltage = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setTurretVoltage(double voltage) {}
}
