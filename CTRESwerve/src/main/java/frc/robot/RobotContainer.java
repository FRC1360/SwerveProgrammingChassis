// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.drive.AimAtTagCommand;
import frc.robot.commands.drive.AlignToTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.orbitSwerveRequests.FieldCentricFacingTag;
import frc.robot.util.DriveConstants;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.MaxSpeed * DriveConstants.joystickDeadbandDecimal).withRotationalDeadband(DriveConstants.MaxAngularRate * DriveConstants.joystickDeadbandDecimal) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final FieldCentricFacingTag driveFacingTag = new FieldCentricFacingTag()
        .withDeadband(DriveConstants.MaxSpeed * DriveConstants.joystickDeadbandDecimal)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withHeadingPID(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd)
        .withAppliedCamera(drivetrain.frontRightSwerveCamera)
        .withTargetTagID(7);

    private final AimAtTagCommand aimAtTagCommandFL = new AimAtTagCommand(
        drivetrain,
        drivetrain.frontLeftSwerveCamera,
        7,
        () -> -joystick.getLeftY() * DriveConstants.MaxSpeed,
        () -> -joystick.getLeftX() * DriveConstants.MaxSpeed,
        true,
        false
    );
    private final AimAtTagCommand aimAtTagCommandFR = new AimAtTagCommand(
        drivetrain,
        drivetrain.frontRightSwerveCamera,
        7,
        () -> -joystick.getLeftY() * DriveConstants.MaxSpeed,
        () -> -joystick.getLeftX() * DriveConstants.MaxSpeed,
        true,
        false
    );

    private final AlignToTagCommand alignToTagCommandFL = new AlignToTagCommand(
        drivetrain,
        drivetrain.frontLeftSwerveCamera,
        7,
        false,
        new Pose2d(0.8, 0.0, new Rotation2d(0.0))
    );
    private final AlignToTagCommand alignToTagCommandFR = new AlignToTagCommand(
        drivetrain,
        drivetrain.frontRightSwerveCamera,
        7,
        false,
        new Pose2d(0.8, 0.0, new Rotation2d(0.0))
    );
    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * DriveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftTrigger(0.9).whileTrue(alignToTagCommandFL);
        joystick.rightTrigger(0.9).whileTrue(alignToTagCommandFR);
        joystick.leftBumper().whileTrue(aimAtTagCommandFL);
        joystick.rightBumper().whileTrue(aimAtTagCommandFR);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        joystick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
