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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

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
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final EightBitDoController driverControllerEightBitDo = new EightBitDoController(1);
  private final CommandXboxController gunnerController = new CommandXboxController(2);
  private final EightBitDoController gunnerControllerEightBitDo = new EightBitDoController(3);

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
        superstructure = new Superstructure(new SuperstructureIO() {});
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
        superstructure = new Superstructure(new SuperstructureIOSim());
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
        superstructure = new Superstructure(new SuperstructureIO() {});
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
            () -> -driverControllerEightBitDo.getLeftY(),
            () -> -driverControllerEightBitDo.getLeftX(),
            () -> {
            if (Math.hypot(driverControllerEightBitDo.getRightX(), driverControllerEightBitDo.getRightY()) > 0.5) {
              boolean isFlipped =
                DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

              return isFlipped
                ? new Rotation2d(Math.atan2(driverControllerEightBitDo.getRightX(), driverControllerEightBitDo.getRightY()))
                : new Rotation2d(Math.atan2(-driverControllerEightBitDo.getRightX(), -driverControllerEightBitDo.getRightY()));
              } else {
                return drive.getRotation();
              }
            }
        )
    );

    driverControllerEightBitDo
      .a()
      .whileTrue(
        DriveCommands.pantryApproachDrive(drive));

    driverControllerEightBitDo
      .pov(0)
      .whileTrue(
        DriveCommands.joystickDriveAtAngle(
          drive,
          () -> 0.5,
          () -> 0.0,
          () -> drive.getRotation()
        )
      );

    driverControllerEightBitDo
      .pov(180)
      .whileTrue(
        DriveCommands.joystickDriveAtAngle(
          drive,
          () -> -0.5,
          () -> 0.0,
          () -> drive.getRotation()
        )
      );
    // Reset gyro to 0° when B button is pressed
    driverController
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

    gunnerControllerEightBitDo
      .leftBumper()
      .whileTrue(
        intake.moveForwards()
      );

    gunnerControllerEightBitDo
      .leftTrigger()
      .whileTrue(
        intake.moveBackwards()
      );

    // //////////////// TURRET COMMANDS /////////////////////////////////
    gunnerControllerEightBitDo
      .x()
      .onTrue(
        turret.setTurretForwards()
      );

    Trigger gunnerLeftJoystickMoved = new Trigger(
      () -> 
        Math.abs(gunnerControllerEightBitDo.getLeftX()) > 0.5
          || Math.abs(gunnerControllerEightBitDo.getLeftY()) > 0.5
    );

    gunnerLeftJoystickMoved.whileTrue(
      turret.setTurretSetpointRadians(
        () ->
          -Math.atan2(
            MathUtil.applyDeadband(
              gunnerControllerEightBitDo.getLeftY(), 
              0.2
            ), // applyDeadband ignores inputs below threshold
            MathUtil.applyDeadband(
              gunnerControllerEightBitDo.getLeftX(), 
              0.2
            )
          )
      )
    );
    // ////////// Superstructure Commands
    
    driverControllerEightBitDo
      .a()
      .onTrue(
        superstructure.superstructureHighestPosition());

    driverControllerEightBitDo
      .rightBumper()
      .onTrue(
        superstructure.superstructureLowestPosition());

    Trigger driverLeftJoystickMoved = new Trigger(
      () -> 
        Math.abs(driverControllerEightBitDo.getLeftX()) > 0.98
          || Math.abs(driverControllerEightBitDo.getLeftY()) > 0.98
    );

    driverLeftJoystickMoved
      .onTrue(
      superstructure.superstructureLowestPosition()
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
