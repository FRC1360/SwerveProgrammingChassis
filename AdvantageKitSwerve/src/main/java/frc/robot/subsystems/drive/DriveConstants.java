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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.7;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(29.5);
  public static final double wheelBase = Units.inchesToMeters(29.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-2.270 + Math.PI);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(2.213 + Math.PI);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(2.802 + Math.PI);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-1.403 + Math.PI);

  // Device CAN IDs
  public static final int pigeonCanId = 5;

  public static final int frontLeftDriveCanId = 10;
  public static final int backLeftDriveCanId = 20;
  public static final int frontRightDriveCanId = 15;
  public static final int backRightDriveCanId = 25;

  public static final int frontLeftTurnCanId = 11;
  public static final int backLeftTurnCanId = 21;
  public static final int frontRightTurnCanId = 16;
  public static final int backRightTurnCanId = 26;

  // Cancoder IDs
  public static final int frontLeftEncoderCanId = 12;
  public static final int backLeftEncoderCanId = 22;
  public static final int frontRightEncoderCanId = 17;
  public static final int backRightEncoderCanId = 27;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit =
      18; // Default is 35A, set to 18A based on slip current calibration
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.285);
  public static final double driveMotorReduction = 6.75;
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.12862;
  public static final double driveKv = 0.13806;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 150.0 / 7.0;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true; // This doesn't do anything??
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 3.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // Pid Constants for alignment
  public static final double tagXKp = 2.0;
  public static final double tagXKi = 0.0;
  public static final double tagXKd = 0.0;

  public static final double tagYKp = 2.0;
  public static final double tagYKi = 0.0;
  public static final double tagYKd = 0.0;

  public static final double tagRKp = 0.2 * 0.6;
  public static final double tagRKi = (1.2 * 0.2) / 0.37;
  public static final double tagRKd = (3 * 0.2 * 0.37) / 40;

  public static final double slewRateConstant = 0.0;

  // PathPlanner configuration
  public static final double robotMassKg = 28.58;
  public static final double robotMOI = 2.6744; // TODO APPROXIMATE VALUE
  public static final double wheelCOF = 1.2; // TODO CHANGE AS ITS DEFAULT VALUE
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
