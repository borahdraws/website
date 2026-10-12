// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {

    public boolean elevatorLeaderConnected = false;
    public boolean elevatorFollowerConnected = false;

    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorLeaderCurrentAmps = 0.0;
    public double elevatorFollowerCurrentAmps = 0.0;

    public double elevatorLeaderTempCelsius = 0.0;
    public double elevatorFollowerTempCelsius = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVoltage(double volts) {}
  
  public default void resetElevatorPosition(double meters) {}

  public default void stopElevator() {}
}
