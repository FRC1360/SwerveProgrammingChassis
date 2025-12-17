// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class DriveCommands {
  private static final double DEADBAND = 0.15;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
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
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
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
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
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
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

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

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  public static Command aimAtTag(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      boolean isFieldRelative) {

    PIDController tagRPidController =
        new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);
    tagRPidController.enableContinuousInput(-180, 180);

    return Commands.run(
        () -> {
          double xSpeed = xSupplier.getAsDouble();
          double ySpeed = ySupplier.getAsDouble();
          double rotSpeed = rotSupplier.getAsDouble();

          boolean targetVisible = false;
          double targetYaw = 0.0;

          List<PhotonPipelineResult> results = drive.frontRightSwerveCamera.getPipelineResults();
          if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                if (target.getFiducialId() == 6) {
                  // Found Tag 7, record its information
                  targetYaw = target.getYaw();
                  targetVisible = true;
                }
              }
            }
          }

          if (targetVisible) {
            rotSpeed = tagRPidController.calculate(targetYaw, 0);
          }
          ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

          if (isFieldRelative) {
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
          } else {
            drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRotation()));
          }
        },
        drive);
  }

  public static Command aimAtPose(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      boolean isFieldRelative) {

    PIDController tagRPidController =
        new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);
    tagRPidController.enableContinuousInput(-180, 180);

    return Commands.run(
        () -> {
          double xSpeed = xSupplier.getAsDouble();
          double ySpeed = ySupplier.getAsDouble();
          double rotSpeed = rotSupplier.getAsDouble();

          double targetYaw = 0.0;

          targetYaw =
              PhotonUtils.getYawToPose(
                      drive.getPose(),
                      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark)
                          .getTagPose(6)
                          .get()
                          .toPose2d())
                  .getDegrees();
          rotSpeed = -tagRPidController.calculate(targetYaw, 0);

          ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

          if (isFieldRelative) {
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
          } else {
            drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRotation()));
          }
        },
        drive);
  }

  public static Command alignToTag(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      Pose2d targetTagOffset, // this is the target pose relative to the tag
      boolean
          isFieldRelative // PID done thru robotrelative, this specifies style of manual override
      ) {

    PIDController tagRPidController =
        new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);
    tagRPidController.enableContinuousInput(-180, 180);

    PIDController tagXPidController =
        new PIDController(DriveConstants.tagXKp, DriveConstants.tagXKi, DriveConstants.tagXKd);

    PIDController tagYPidController =
        new PIDController(DriveConstants.tagYKp, DriveConstants.tagYKi, DriveConstants.tagYKd);

    return Commands.run(
        () -> {
          double xSpeed = xSupplier.getAsDouble();
          double ySpeed = ySupplier.getAsDouble();
          double rotSpeed = rotSupplier.getAsDouble();

          double speedRotationAngle = Math.toRadians(135);
          double rotatedXSpeed = 0.0;
          double rotatedYSpeed = 0.0;

          boolean targetVisible = false;
          double targetX = 0.0;
          double targetY = 0.0;
          double targetYaw = 0.0;

          List<PhotonPipelineResult> results = drive.frontRightSwerveCamera.getPipelineResults();
          if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                if (target.getFiducialId() == 6) {
                  // Found Tag 7, record its information
                  targetX = target.getBestCameraToTarget().getX();
                  targetY = target.getBestCameraToTarget().getY();
                  targetYaw = target.getBestCameraToTarget().getRotation().getAngle();
                  targetVisible = true;
                }
              }
            }
          }

          if (targetVisible) {
            rotSpeed = tagRPidController.calculate(Math.toDegrees(targetYaw), 180);
            ySpeed = -tagYPidController.calculate(targetY, 0.0);
            xSpeed = -tagXPidController.calculate(targetX, 0.5);
          }

          rotatedXSpeed =
              (xSpeed * Math.cos(speedRotationAngle)) - (ySpeed * Math.sin(speedRotationAngle));
          rotatedYSpeed =
              (xSpeed * Math.sin(speedRotationAngle)) + (ySpeed * Math.cos(speedRotationAngle));

          ChassisSpeeds speeds = new ChassisSpeeds(rotatedXSpeed, rotatedYSpeed, rotSpeed);
          drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRotation()));
        },
        drive);
  }

  public static Command alignToTagStrafe(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      // Pose2d targetTagOffset, // this is the target pose relative to the tag
      boolean
          isFieldRelative // PID done thru robotrelative, this specifies style of manual override
      ) {

    ProfiledPIDController tagRPidController =
        new ProfiledPIDController(
            DriveConstants.tagRKp,
            DriveConstants.tagRKi,
            DriveConstants.tagRKd,
            new TrapezoidProfile.Constraints(1, 1),
            0.02);
    // tagRPidController.enableContinuousInput(-180, 180);

    PIDController tagXPidController = new PIDController(5.0, 0.0, 0.0);

    PIDController tagYPidController = new PIDController(0.035, 0.0, 0.0);

    SmartDashboard.putData("Heading Controller", tagRPidController);
    SmartDashboard.putData("Circular Motion Controller", tagYPidController);
    SmartDashboard.putData("Radius Distance Controller", tagXPidController);

    return Commands.run(
        () -> {
          double xSpeed = xSupplier.getAsDouble();
          double ySpeed = ySupplier.getAsDouble();
          double rotSpeed = rotSupplier.getAsDouble();

          double speedRotationAngle = Math.toRadians(135);
          double rotatedXSpeed = 0.0;
          double rotatedYSpeed = 0.0;

          boolean targetVisible = false;
          double targetX = 0.0;
          double targetY = 0.0;
          double targetYaw = 0.0;

          List<PhotonPipelineResult> results = drive.frontRightSwerveCamera.getPipelineResults();
          if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                if (target.getFiducialId() == 6) {
                  targetX =
                      Math.sqrt(
                          (target.getBestCameraToTarget().getX()
                                  * target.getBestCameraToTarget().getX())
                              + (target.getBestCameraToTarget().getY()
                                  * target.getBestCameraToTarget().getY()));
                  targetY = target.getBestCameraToTarget().getRotation().getAngle();
                  targetYaw = target.getYaw();
                  targetVisible = true;
                }
              }
            }
          }

          if (targetVisible) {
            rotSpeed = tagRPidController.calculate(targetYaw, 0);
            ySpeed = -tagYPidController.calculate(Math.toDegrees(targetY), 180);
            xSpeed = -tagXPidController.calculate(targetX, 0.5);

            // Deadband for jerk compensation when at setpoint
            // if (Math.abs(rotSpeed) < 0.05) rotSpeed = 0;
            // if (Math.abs(xSpeed) < 0.05) xSpeed = 0;
            // if (Math.abs(ySpeed) < 0.05) ySpeed = 0;

            SmartDashboard.putNumber("Heading Target", 0);

            SmartDashboard.putNumber("Arc Current", Math.toDegrees(targetY));
            SmartDashboard.putNumber("Arc Target", 180);

            SmartDashboard.putNumber("Radius Current", targetX);
            SmartDashboard.putNumber("Radius Target", 0.5);
          }

          SmartDashboard.putNumber(
              "Heading Current Position", tagRPidController.getSetpoint().position);
          SmartDashboard.putNumber(
              "Heading Current Velocity", tagRPidController.getSetpoint().velocity);

          rotatedXSpeed =
              (xSpeed * Math.cos(speedRotationAngle)) - (ySpeed * Math.sin(speedRotationAngle));
          rotatedYSpeed =
              (xSpeed * Math.sin(speedRotationAngle)) + (ySpeed * Math.cos(speedRotationAngle));

          ChassisSpeeds speeds = new ChassisSpeeds(rotatedXSpeed, rotatedYSpeed, rotSpeed);
          drive.runVelocity(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRotation()));
        },
        drive);
  }

  public static Command alignToTagHolo(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier,
      // Pose2d targetTagOffset, // this is the target pose relative to the tag
      boolean
          isFieldRelative // PID done thru robotrelative, this specifies style of manual override
      ) {

    HolonomicDriveController holoController =
        new HolonomicDriveController(
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new ProfiledPIDController(4, 0.0, 0.1, new TrapezoidProfile.Constraints(360, 360)));
    holoController.setTolerance(new Pose2d(0.015, 0.015, Rotation2d.fromDegrees(3)));

    StructPublisher<Pose2d> tagPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/TagPose", Pose2d.struct)
            .publish();
    StructPublisher<Pose2d> robotPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/RobotPose", Pose2d.struct)
            .publish();

    return Commands.run(
        () -> {
          ChassisSpeeds robotSpeeds =
              new ChassisSpeeds(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotSupplier.getAsDouble());
          double cameraRotationAngle = -45;

          Pose2d tagPose = new Pose2d();
          Pose2d robotPose = new Pose2d(0.3, -0.3, Rotation2d.fromDegrees(0));

          double targetYaw = 0.0;
          boolean targetVisible = false;

          List<PhotonPipelineResult> results = drive.frontRightSwerveCamera.getPipelineResults();
          if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
              // At least one AprilTag was seen by the camera
              for (var target : result.getTargets()) {
                if (target.getFiducialId() == 6) {
                  // targetX =
                  //     Math.sqrt(
                  //         (target.getBestCameraToTarget().getX()
                  //                 * target.getBestCameraToTarget().getX())
                  //             + (target.getBestCameraToTarget().getY()
                  //                 * target.getBestCameraToTarget().getY()));
                  // targetY = target.getBestCameraToTarget().getRotation().getAngle();
                  // targetYaw = target.getYaw();
                  // targetVisible = true;
                  tagPose =
                      new Pose2d(
                          target.getBestCameraToTarget().getX(),
                          target.getBestCameraToTarget().getY(),
                          target.getBestCameraToTarget().getRotation().toRotation2d());
                  targetYaw = target.getYaw();
                  targetVisible = true;
                }
              }
            }
          }

          if (targetVisible) {
            tagPose = tagPose.rotateBy(Rotation2d.fromDegrees(cameraRotationAngle));
            // robotPose = robotPose.rotateBy(Rotation2d.fromDegrees(cameraRotationAngle));

            robotSpeeds =
                holoController.calculate(
                    robotPose,
                    tagPose,
                    0.0,
                    Rotation2d.fromDegrees(targetYaw)
                        .minus(tagPose.getRotation())
                        .plus(Rotation2d.fromDegrees(180))
                        .plus(Rotation2d.fromDegrees(cameraRotationAngle)));

            SmartDashboard.putNumber("Holonomic Controller/X Target", robotPose.getX());
            SmartDashboard.putNumber("Holonomic Controller/Y Target", robotPose.getY());
            SmartDashboard.putNumber(
                "Holonomic Controller/Yaw Target", robotPose.getRotation().getDegrees());

            SmartDashboard.putNumber("Holonomic Controller/X Current", tagPose.getX());
            SmartDashboard.putNumber("Holonomic Controller/Y Current", tagPose.getY());
            SmartDashboard.putNumber(
                "Holonomic Controller/Yaw Current",
                Rotation2d.fromDegrees(targetYaw)
                    .minus(tagPose.getRotation())
                    .plus(Rotation2d.fromDegrees(180))
                    .plus(Rotation2d.fromDegrees(cameraRotationAngle))
                    .getDegrees());

            SmartDashboard.putBoolean("Holonomic Controller/At Goal", holoController.atReference());

            tagPosePublisher.accept(tagPose);
            robotPosePublisher.accept(robotPose);

            if (holoController.atReference()) robotSpeeds = new ChassisSpeeds();
          }

          drive.runVelocity(
              ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds.times(-1), drive.getRotation()));
        },
        drive);
  }
}
