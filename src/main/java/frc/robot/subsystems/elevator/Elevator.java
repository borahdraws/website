// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public enum Goal {
    LOWEST(
      0.0
    ),
    HIGHEST(
      Units.inchesToMeters(14.0)
    );

    final double elevatorCarriageTargetHeightMeters;

    Goal(double elevatorHeightMeters) {
      this.elevatorCarriageTargetHeightMeters = elevatorHeightMeters;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Goal goal = Goal.LOWEST;

  private final ProfiledPIDController elevatorPID =
    new ProfiledPIDController(
      80.0, 
      0.0, 
      0.2, 
      new TrapezoidProfile.Constraints(
        8.0,
        6.0
      )
    );

  private final ElevatorFeedforward elevatorFF = 
    new ElevatorFeedforward(
      0.1, 
      0.3, 
      4.0, 
      0.05
    );

  // //// MECHANISM 2D /////////////////////////////////////////////////////////

  private final LoggedMechanism2d elevatorMechanismContainer =
    new LoggedMechanism2d(
      1.5,
      2.0
    );

  private final LoggedMechanismRoot2d elevatorBaseRoot =
    elevatorMechanismContainer.getRoot(
      "Elevator Anchor Point",
      0.75,
      0.0
    );
  private final LoggedMechanismLigament2d elevatorBaseMechanism =
    elevatorBaseRoot.append(
      new LoggedMechanismLigament2d(
        "Elevator Base",
        Units.inchesToMeters(24.0),
        90.0,
        6.0,
        new Color8Bit(Color.kRed)
      )
    );

  private final LoggedMechanismRoot2d elevatorCarriageRoot =
    elevatorMechanismContainer.getRoot(
      "Elevator Carriage Anchor Point", 
      0.75, 
      Units.inchesToMeters(2.0)
    );

  private final LoggedMechanismLigament2d elevatorCarriageMechanism =
    elevatorCarriageRoot.append(
      new LoggedMechanismLigament2d(
        "Elevator Carriage",
        Units.inchesToMeters(24.0), 
        90,
        3.0,
        new Color8Bit(Color.kOrange)
      )
    );

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    SmartDashboard.putData("Elevator/Mechanism2d", elevatorMechanismContainer);

    elevatorPID.setTolerance(Units.inchesToMeters(5.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double elevatorPidOutput = elevatorPID.calculate(inputs.elevatorPositionMeters, goal.elevatorCarriageTargetHeightMeters);
    double elevatorFFOutput = elevatorFF.calculate(elevatorPID.getSetpoint().velocity);
    io.setElevatorVoltage(elevatorPidOutput + elevatorFFOutput);

    elevatorCarriageRoot.setPosition(
      0.8, 
      0.1 + inputs.elevatorPositionMeters
    );

    Logger.recordOutput("Elevator/Mechanism", elevatorMechanismContainer);
    Logger.recordOutput("Elevator/MechanismPoses", elevatorMechanismContainer.generate3dMechanism().toArray(new Pose3d[0]));

    Logger.recordOutput("Elevator/Goal", goal.toString());
    Logger.recordOutput("Elevator/AtGoal", atElevatorGoal());
  }

  public void setElevatorGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getElevatorGoal(){
    return goal;
  }

  public boolean atElevatorGoal() {
    return elevatorPID.atGoal();
  }

  // //////////////////// COMMANDS

  public Command setElevatorGoalCommand(Goal goal) {
    return runOnce(
      () -> {
        setElevatorGoal(goal);
      }
    ); 
  }

  public Command elevatorHighestPosition() {
    return setElevatorGoalCommand(Goal.HIGHEST)
      .withName("Elevator Highest Position");
  }

  public Command elevatorLowestPosition() {
    return setElevatorGoalCommand(Goal.LOWEST)
      .withName("Elevator Lowest Position");
  }
}