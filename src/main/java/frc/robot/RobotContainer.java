// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Turret turret;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController gunnerController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOTalonFX());
        turret = new Turret(new TurretIO() {}); // *** placeholder
        vision = 
          new Vision(
              drive::addVisionMeasurement,
              new VisionIO() {}
          );
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        turret = new Turret(new TurretIOSim());
        vision = 
          new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
              camera0Name,
              robotToCamera0,
              drive::getPose
            )
          );
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        turret = new Turret(new TurretIO() {});
        vision = 
          new Vision(
              drive::addVisionMeasurement,
              new VisionIO() {}
          );
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    // //////////////// DRIVE COMMANDS /////////////////////////////////
    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> {
              Logger.recordOutput("Debug/RightStickX", controller.getRightX());
              Logger.recordOutput("Debug/RightStickY", controller.getRightY());
              Logger.recordOutput("Debug/RawAxis3", controller.getHID().getRawAxis(3));
              Logger.recordOutput("Debug/RawAxis4", controller.getHID().getRawAxis(4));
              if (Math.hypot(controller.getHID().getRawAxis(3), controller.getHID().getRawAxis(4)) > 0.5) {
                return new Rotation2d(Math.atan2(controller.getHID().getRawAxis(3),controller.getHID().getRawAxis(4)));
              } else {
                return drive.getRotation();
              }
            }
        )
    );

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // //////////////// INTAKE COMMANDS /////////////////////////////////
    intake
      .setDefaultCommand(
        intake.Stop()
      );

    gunnerController
      .button(EIGHT_BIT_DO_LEFT_BUMPER)
      .whileTrue(
        intake.moveForwards()
      );

    gunnerController
      .button(EIGHT_BIT_DO_LEFT_TRIGGER)
      .whileTrue(
        intake.moveBackwards()
      );

    // //////////////// TURRET COMMANDS /////////////////////////////////
    gunnerController
      .button(EIGHT_BIT_DO_X)
      .onTrue(
        turret.setTurretForwards()
      );

    Trigger leftJoystickMoved = new Trigger(
      () ->
        Math.abs(gunnerController.getLeftX()) > 0.2 || 
          Math.abs(gunnerController.getLeftY()) > 0.2
    );

    leftJoystickMoved.whileTrue(
      turret.setTurretSetpointRadians(
        () ->
          -Math.atan2(
            MathUtil.applyDeadband(
              gunnerController.getLeftY(), 
              0.2
            ), // applyDeadband ignores inputs below threshold
            MathUtil.applyDeadband(
              gunnerController.getLeftX(), 
              0.2
            )
          )
      )
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
