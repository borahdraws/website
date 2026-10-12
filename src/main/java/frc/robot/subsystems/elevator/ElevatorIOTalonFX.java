// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {

  // ---- TODO: set these to match your robot ----
  private static final int LEADER_ID = 10; // left motor CAN ID
  private static final int FOLLOWER_ID = 11; // right motor CAN ID
  private static final double GEAR_RATIO = 5.0; // motor rotations per drum rotation
  private static final double DRUM_RADIUS_METERS = 0.0254; // radius of the spool the cable wraps on
  private static final double CARRIAGE_MULTIPLIER = 1.0; // 1.0 for a single-stage elevator, 2.0 for a continuous 2-stage, etc.
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  // Meters of carriage travel for one full rotation of the drum
  private static final double METERS_PER_DRUM_ROTATION =
      2.0 * Math.PI * DRUM_RADIUS_METERS * CARRIAGE_MULTIPLIER;

  private final TalonFX leader;
  private final TalonFX follower;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
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
    follower = new TalonFX(FOLLOWER_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Flip this to Clockwise_Positive if the elevator moves down on positive voltage
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // With this set, position/velocity are reported in DRUM rotations, not motor rotations
    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
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

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVolts,
        leaderCurrent,
        followerCurrent,
        leaderTemp,
        followerTemp);
    // Frees up CAN bandwidth by turning off signals we didn't ask for
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, leaderCurrent, leaderTemp);
    var followerStatus = BaseStatusSignal.refreshAll(followerCurrent, followerTemp);

    inputs.elevatorLeaderConnected = leaderStatus.isOK();
    inputs.elevatorFollowerConnected = followerStatus.isOK();

    inputs.elevatorPositionMeters = position.getValue().in(Rotations) * METERS_PER_DRUM_ROTATION;
    inputs.elevatorVelocityMetersPerSec =
        velocity.getValue().in(RotationsPerSecond) * METERS_PER_DRUM_ROTATION;
    inputs.elevatorAppliedVolts = appliedVolts.getValue().in(Volts);
    inputs.elevatorLeaderCurrentAmps = leaderCurrent.getValue().in(Amps);
    inputs.elevatorFollowerCurrentAmps = followerCurrent.getValue().in(Amps);
    inputs.elevatorLeaderTempCelsius = leaderTemp.getValue().in(Celsius);
    inputs.elevatorFollowerTempCelsius = followerTemp.getValue().in(Celsius);
  }

  @Override
  public void setElevatorVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
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