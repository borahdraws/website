// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.*;

public class Superstructure extends SubsystemBase {
  public enum Goal {
    LOWEST(
      ELEVATOR_CARRIAGE_TARGET_HEIGHT_INITIAL,
      Units.degreesToRadians(-10.0)
    ),
    MIDDLE(
      ELEVATOR_CARRIAGE_HEIGHT_LEVEL_1_SHELF,
      Units.degreesToRadians(10.0)
    ),
    HIGHEST(
      ELEVATOR_CARRIAGE_HEIGHT_LEVEL_2_SHELF, 
      Units.degreesToRadians(30.0)
    );
 
    final double elevatorCarriageTargetHeightMeters;
    final double armTargetAngleRad;
 
    Goal(double elevatorHeightMeters, double armAngleRad) {
      this.elevatorCarriageTargetHeightMeters = elevatorHeightMeters;
      this.armTargetAngleRad = armAngleRad;
    }
  }

  private final SuperstructureIO io;
  private final SuperstructureIOInputsAutoLogged inputs = new SuperstructureIOInputsAutoLogged();

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

  private final ProfiledPIDController armPID =
    new ProfiledPIDController(
      30.0,
      0.0,
      0.2,
      new TrapezoidProfile.Constraints(
        2.0,
        2.0
      )
    );

  private final ElevatorFeedforward elevatorFF = 
    new ElevatorFeedforward(
      0.1, 
      0.3, 
      4.0, 
      0.05
    );

  private final ArmFeedforward armFF = 
    new ArmFeedforward(
      0.1, 
      0.4,
      1.0,
      0.02
    );

  // //// MECHANISM 2D /////////////////////////////////////////////////////////

  private static final double ARM_VISUAL_BASE_DEGREES = -90.0;

  private final LoggedMechanism2d superstructureMechanismContainer =
    new LoggedMechanism2d(
      1.5,
      2.0
    );

  private final LoggedMechanismRoot2d elevatorBaseRoot =
    superstructureMechanismContainer.getRoot(
      "Superstructure Anchor Point",
      0.75,
      0.0
    );
  private final LoggedMechanismLigament2d elevatorBaseMechanism =
    elevatorBaseRoot.append(
      new LoggedMechanismLigament2d(
        "Elevator Base",
        ELEVATOR_BASE_LENGTH,
        90.0,
        6.0,
        new Color8Bit(Color.kRed)
      )
    );

  private final LoggedMechanismRoot2d elevatorCarriageRoot =
    superstructureMechanismContainer.getRoot(
      "Elevator Carriage Anchor Point", 
      0.75, 
      Units.inchesToMeters(2.0)
    );
  private final LoggedMechanismLigament2d elevatorCarriageMechanism =
    elevatorCarriageRoot.append(
      new LoggedMechanismLigament2d(
        "Elevator Carriage",
        ELEVATOR_CARRIAGE_LENGTH, 
        90,
        3.0,
        new Color8Bit(Color.kOrange)
      )
    );

  private final LoggedMechanismLigament2d armMechanism =
      elevatorCarriageMechanism.append(
        new LoggedMechanismLigament2d(
          "Arm", 
          ARM_LENGTH, 
          ARM_VISUAL_BASE_DEGREES,
          3.0,
          new Color8Bit(Color.kGreenYellow)
        )
      );

  /** Creates a new Superstructure. */
  public Superstructure(SuperstructureIO io) {
    this.io = io;

    SmartDashboard.putData("Superstructure/Mechanism2d", superstructureMechanismContainer);

    elevatorPID.setTolerance(Units.inchesToMeters(5.0));
    armPID.setTolerance(Units.degreesToRadians(2.0));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    io.updateInputs(inputs);
    Logger.processInputs("Superstructure", inputs);
    
    double elevatorPidOutput = elevatorPID.calculate(inputs.elevatorPositionMeters, goal.elevatorCarriageTargetHeightMeters);
    double elevatorFFOutput = elevatorFF.calculate(elevatorPID.getSetpoint().velocity);
    io.setElevatorVoltage(elevatorPidOutput + elevatorFFOutput);
 
    double armPidOutput = armPID.calculate(inputs.armPositionRad, goal.armTargetAngleRad);
    double armFFOutput = armFF.calculate(armPID.getSetpoint().position, armPID.getSetpoint().velocity);
    io.setArmVoltage(armPidOutput + armFFOutput);
 
    elevatorCarriageRoot.setPosition(
      0.8, 
      0.1 + inputs.elevatorPositionMeters
    );
    armMechanism.setAngle(ARM_VISUAL_BASE_DEGREES + Units.radiansToDegrees(inputs.armPositionRad));

    Logger.recordOutput("Superstructure/Mechanism", superstructureMechanismContainer);
    Logger.recordOutput("Superstructure/MechanismPoses", superstructureMechanismContainer.generate3dMechanism().toArray(new Pose3d[0]));
 
    Logger.recordOutput("Superstructure/Goal", goal.toString());
    Logger.recordOutput("Superstructure/AtGoal", atSuperstructureGoal());
  }

  public void setSuperstructureGoal(Goal goal) {
    this.goal = goal;
  }

  public Goal getSuperstructureGoal(){
    return goal;
  }

  public boolean atSuperstructureGoal() {
    return elevatorPID.atGoal() && armPID.atGoal();
  }

  // //////////////////// COMMANDS

  public Command setSuperstructureGoalCommand(Goal goal) {
    return runOnce(
      () -> {
        setSuperstructureGoal(goal);
      }
    ); 
  }

  public Command superstructureHighestPosition() {
    return setSuperstructureGoalCommand(Goal.HIGHEST)
      .withName("Superstructure Highest Position");
  }

  public Command superstructureMiddlePosition() {
    return setSuperstructureGoalCommand(Goal.MIDDLE)
      .withName("Superstructure Middle Position");
  }

  public Command superstructureLowestPosition() {
    return setSuperstructureGoalCommand(Goal.LOWEST)
      .withName("Superstructure Lowest Position");
  }
}
