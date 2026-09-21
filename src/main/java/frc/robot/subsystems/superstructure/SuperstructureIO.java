// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SuperstructureIO {
  @AutoLog
  public static class SuperstructureIOInputs {
    // Elevator
    public boolean elevatorLeaderConnected = false;
    public boolean elevatorFollowerConnected = false;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorLeaderCurrentAmps = 0.0;
    public double elevatorFollowerCurrentAmps = 0.0;
    public double elevatorLeaderTempCelsius = 0.0;
    public double elevatorFollowerTempCelsius = 0.0;

    // Arm
    public boolean armConnected = false;
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
  }

  public default void updateInputs(SuperstructureIOInputs inputs) {}

  public default void setElevatorVoltage(double voltage) {}

  public default void setArmVoltage(double voltage) {}

  public default void stopSuperstructure() {}
}
