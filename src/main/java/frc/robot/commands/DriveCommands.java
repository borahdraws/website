// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 10.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static final double ROBOT_HALF_LENGTH_METERS = Units.inchesToMeters(30.0/2.0);
  private static final Rotation2d PANTRY_SNAP_DIRECTION = Rotation2d.fromDegrees(90);
  private static final double PANTRY_STANDOFF_METERS = Units.feetToMeters(2.0);

  private static final Translation2d PANTRY_LEFT_CORNER_BLUE = 
    VisionConstants
    .aprilTagLayout
      .getTagPose(7)
        .get() // make sure the ID is in the AprilTags layout file
        .getTranslation() // converts TagPose to 3d coords
        .toTranslation2d(); // converts 3D coords to 2d coords

  private static final Translation2d PANTRY_RIGHT_CORNER_BLUE = 
    VisionConstants
    .aprilTagLayout
      .getTagPose(6)
        .get() // make sure the ID is in the AprilTags layout file
        .getTranslation() // converts TagPose to 3d coords
        .toTranslation2d(); // converts 3D coords to 2d coords

  private static final Translation2d PANTRY_LEFT_CORNER_RED = 
    VisionConstants
    .aprilTagLayout
      .getTagPose(5)
        .get() // make sure the ID is in the AprilTags layout file
        .getTranslation() // converts TagPose to 3d coords
        .toTranslation2d(); // converts 3D coords to 2d coords

  private static final Translation2d PANTRY_RIGHT_CORNER_RED = 
    VisionConstants
    .aprilTagLayout
      .getTagPose(4)
        .get() // make sure the ID is in the AprilTags layout file
        .getTranslation() // converts TagPose to 3d coords
        .toTranslation2d(); // converts 3D coords to 2d coords
  
  private static Pose2d computePantryApproachPose(Drive drive, boolean isFlipped) {
    Translation2d leftCorner = isFlipped ? PANTRY_LEFT_CORNER_RED : PANTRY_LEFT_CORNER_BLUE;
    Translation2d rightCorner = isFlipped ? PANTRY_RIGHT_CORNER_RED : PANTRY_RIGHT_CORNER_BLUE;

    double minX = Math.min(leftCorner.getX(), rightCorner.getX());
    double maxX = Math.max(leftCorner.getX(), rightCorner.getX());
    double wallY = leftCorner.getY();

    double robotX = drive.getPose().getX();
    double closestX = MathUtil.clamp(robotX, minX, maxX); // this is the key line — closest point, not midpoint
    double approachY = wallY - ROBOT_HALF_LENGTH_METERS - PANTRY_STANDOFF_METERS; // stand this far off the wall, on the field side

    return new Pose2d(new Translation2d(closestX, approachY), PANTRY_SNAP_DIRECTION);
  }

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */   
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController rotationPID =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
      () -> {
        // Get linear velocity
        Translation2d linearVelocity =
            getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        
        // Calculate angular speed
        double omega =
            rotationPID.calculate(
                drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds =
            new ChassisSpeeds(
                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega);

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
                
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                isFlipped
                    ? drive.getRotation().plus(new Rotation2d(Math.PI))
                    : drive.getRotation()));
      },
      drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> rotationPID.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors. 
   * This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
    
  }

  public static Command driveToPose(Drive drive, Pose2d targetPose) {
    return new DriveToPoseCommand(drive, targetPose);
  }

  private static class DriveToPoseCommand extends Command {
    private final Drive drive;
    private final Pose2d targetPose;
    private final PIDController xController = new PIDController(3.0, 0, 0);
    private final PIDController yController = new PIDController(3.0, 0, 0);
    private final PIDController thetaController = new PIDController(4.0, 0, 0);

    DriveToPoseCommand(Drive drive, Pose2d targetPose) {
      this.drive = drive;
      this.targetPose = targetPose;
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      xController.setTolerance(0.03);
      yController.setTolerance(0.03);
      thetaController.setTolerance(Math.toRadians(2));
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      xController.reset();
      yController.reset();
      thetaController.reset();
    }
    
    @Override
    public void execute() {
      Pose2d pose = drive.getPose();
      double vx = xController.calculate(pose.getX(), targetPose.getX());
      double vy = yController.calculate(pose.getY(), targetPose.getY());
      double omega = thetaController.calculate(pose.getRotation().getRadians(), targetPose.getRotation().getRadians());
      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, pose.getRotation()));
    }

    @Override
    public boolean isFinished() {
      return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
      drive.stop();
    }
  }

  public static Command pantryApproachDrive(Drive drive) {
    return Commands.defer(
      () -> {
        boolean isFlipped =
            DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        return driveToPose(drive, computePantryApproachPose(drive, isFlipped));
      },
      java.util.Set.of(drive));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
