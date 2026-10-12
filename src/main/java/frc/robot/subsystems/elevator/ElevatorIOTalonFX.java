// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ElevatorIOTalonFX implements ElevatorIO {

  private static final int LEADER_ID = 6; // left motor CAN ID
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  private static final double METERS_PER_DRUM_ROTATION = 2.0 * Math.PI * Units.inchesToMeters(0.5);

  private final TalonFX leader;
  private final TalonFX follower;

  private final VoltageOut request = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public ElevatorIOTalonFX() {
    leader = new TalonFX(LEADER_ID);
    follower = new TalonFX(7);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Flip this to Clockwise_Positive if the elevator moves down on positive voltage
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // With this set, position/velocity are reported in DRUM rotations, not motor rotations
    config.Feedback.SensorToMechanismRatio = 5.0;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Follower mirrors the leader. Use Opposed if the motors face each other
    // (typical for left/right), Aligned if they spin the same direction.
    follower.setControl(new Follower(LEADER_ID, MotorAlignmentValue.Opposed));

    leader.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPositionMeters = 
      leader.getPosition().getValue().in(Rotations) * METERS_PER_DRUM_ROTATION;
    inputs.elevatorVelocityMetersPerSec =
        leader.getVelocity().getValue().in(RotationsPerSecond) * METERS_PER_DRUM_ROTATION;
    leader.getMotorVoltage().getValueAsDouble();
    inputs.elevatorLeaderCurrentAmps = leader.getSupplyCurrent().getValueAsDouble();
    inputs.elevatorFollowerCurrentAmps = follower.getSupplyCurrent().getValueAsDouble();
    inputs.elevatorLeaderTempCelsius = leader.getDeviceTemp().getValueAsDouble();
    inputs.elevatorFollowerTempCelsius = follower.getDeviceTemp().getValueAsDouble();
    inputs.elevatorLeaderConnected = leader.isConnected();
    inputs.elevatorFollowerConnected = follower.isConnected();
  }

  @Override
  public void setElevatorVoltage(double volts) {
    leader.setControl(request.withOutput(volts));
  }

  @Override
  public void resetElevatorPosition(double meters) {
    leader.setPosition(meters / METERS_PER_DRUM_ROTATION);
  }

  @Override
  public void stopElevator() {
    leader.setControl(neutralRequest);
  }
}