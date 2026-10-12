// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  private static final int LEADER_ID = 6; // left motor CAN ID
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  private static final double METERS_PER_DRUM_ROTATION = 2.0 * Math.PI * Units.inchesToMeters(0.5);

  private final TalonFX leader;
  private final TalonFX follower;

  private final VoltageOut request = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;
  private final StatusSignal<Temperature> leaderTemp;
  private final StatusSignal<Temperature> followerTemp;

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

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    leaderCurrent = leader.getSupplyCurrent();
    followerCurrent = follower.getSupplyCurrent();
    leaderTemp = leader.getDeviceTemp();
    followerTemp = follower.getDeviceTemp();

  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPositionMeters = position.getValue().in(Rotations) * METERS_PER_DRUM_ROTATION;
    inputs.elevatorVelocityMetersPerSec =
        velocity.getValue().in(RotationsPerSecond) * METERS_PER_DRUM_ROTATION;
    inputs.elevatorAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.elevatorLeaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.elevatorFollowerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.elevatorLeaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.elevatorFollowerTempCelsius = followerTemp.getValueAsDouble();
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