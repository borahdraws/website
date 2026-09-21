// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  PIDController turretPID =
    new PIDController(
      1.0,
      0.0,
      0.06
    );

  Mechanism2d turretMechanismContainer = 
    new Mechanism2d(
      5.0,
      5.0
    );
  
  MechanismRoot2d turretRoot = 
    turretMechanismContainer.getRoot(
      "Turret Anchor Point", 
      2.5, 
      2.5
    );

  MechanismLigament2d turretMechanism =
    turretRoot.append(
      new MechanismLigament2d(
        "Turret", 
        1.0, 
        0.0,
        6.0,
        new Color8Bit(Color.kPurple)
      )
    );

  MechanismLigament2d targetMechanism = 
    turretRoot.append(
      new MechanismLigament2d(
        "Target the turret aims for", 
        0.7, 
        0.0,
        3.0,
        new Color8Bit(Color.kRed)
      )
    );

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;
    
    SmartDashboard.putData("Turret Mechanism Container", turretMechanismContainer);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    targetMechanism.setAngle(Units.radiansToDegrees(turretPID.getSetpoint()));
    turretMechanism.setAngle(Units.radiansToDegrees(inputs.turretAngleRadians));

    io.setTurretVoltage(turretPID.calculate(inputs.turretAngleRadians));
  }

  public Command moveForwards() {
    return run(
      () -> {
        io.setTurretVoltage(5.0);
      });
  }

  public Command moveBackwards() {
    return run(
      () -> {
        io.setTurretVoltage(-5.0);
      });
  }

  public Command Stop() {
    return runOnce(
      () -> {
        io.setTurretVoltage(0);
      });
  }

  public Command setTurretSetpointRadians(DoubleSupplier angle) {
    return run(
      () -> {
        turretPID.setSetpoint(angle.getAsDouble());
      });
  }
 
  public Command setTurretForwards() {
    return run(
      () -> {
        turretPID.setSetpoint(Units.degreesToRadians(90.0)); 
      });
  }

}
