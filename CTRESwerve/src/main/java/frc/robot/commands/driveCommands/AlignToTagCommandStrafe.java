// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveCamera;
import frc.robot.util.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommandStrafe extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveCamera camera;
    private final int tagId;
    private final boolean hasEndCondition;

    private final HolonomicDriveController holoController;
    private final PIDController xController;
    private final PIDController yController;

    private final SwerveRequest.RobotCentric driveRequest;
    private final Pose2d robotToAprilTagOffset;

    private final DoubleSupplier tempXSupplier;
    private final DoubleSupplier tempYSupplier;
    
    private final StructPublisher<Pose2d> tagPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/TagPose", Pose2d.struct)
            .publish();
    
    private final StructPublisher<Pose2d> robotPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/RobotPose", Pose2d.struct)
            .publish();

    /** Creates a new aimAtTag. */
    public AlignToTagCommandStrafe(
        CommandSwerveDrivetrain drivetrain,
        SwerveCamera camera,
        int tagId,
        boolean hasEndCondition,
        Pose2d robotToAprilTagOffset,
        DoubleSupplier tempXSupplier,
        DoubleSupplier tempYSupplier
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.tagId = tagId;
        this.hasEndCondition = hasEndCondition;
        this.robotToAprilTagOffset = robotToAprilTagOffset;

        this.tempXSupplier = tempXSupplier;
        this.tempYSupplier = tempYSupplier;

        this.holoController = new HolonomicDriveController(
            new PIDController(0.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0),
            new ProfiledPIDController(
                5.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(DriveConstants.MaxAngularSpeedRadians, DriveConstants.MaxAngularAccelerationRadians)
            )
        );
        this.xController = new PIDController(3.0, 0.0, 0.0);
        this.yController = new PIDController(3.0, 0.0, 0.0);

        this.driveRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        holoController.setTolerance(DriveConstants.holonomicTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PhotonTrackedTarget aprilTag = new PhotonTrackedTarget();
        boolean targetVisible = false;
        Pose2d currentRobotPose = new Pose2d();
        ChassisSpeeds outputRobotSpeeds;
        Translation2d robotVelocityVector = new Translation2d(
            tempXSupplier.getAsDouble(),
            tempYSupplier.getAsDouble()
        );
        Transform3d robotToCamera = camera.getRobotToCamera();

        List<PhotonPipelineResult> results = camera.getPipelineResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == tagId) {
                        aprilTag = target;
                        targetVisible = true;
                    }
                }
            }
        }

        if (targetVisible) {
            currentRobotPose = new Pose2d(
                aprilTag.getBestCameraToTarget().getX(),
                aprilTag.getBestCameraToTarget().getY(),
                Rotation2d.fromDegrees(180.0)
                    .minus(Rotation2d.fromRadians(robotToCamera.getRotation().getZ()))
            );
            currentRobotPose = currentRobotPose.rotateBy(
                Rotation2d.fromDegrees(180)
                .minus(Rotation2d.fromRadians(aprilTag.getBestCameraToTarget().getRotation().getZ()))
            );

            SmartDashboard.putNumber("April Tag Theta", 
                Units.radiansToDegrees(aprilTag.getBestCameraToTarget().getRotation().getZ())
            );
            SmartDashboard.putNumber("April Tag Yaw", 
                aprilTag.getYaw()
            );
            SmartDashboard.putNumber("April Tag dx", 
                aprilTag.getBestCameraToTarget().getX()
            );
            SmartDashboard.putNumber("April Tag dy", 
                aprilTag.getBestCameraToTarget().getY()
            );

            SmartDashboard.putNumber("Target Heading", 
                Rotation2d.fromRadians(aprilTag.getBestCameraToTarget().getRotation().getZ())
                .plus(Rotation2d.fromRadians(robotToCamera.getRotation().getZ()))
                .times(-1)
                .minus(Rotation2d.fromDegrees(aprilTag.getYaw()))
                .getDegrees()
            );
            
            robotPosePublisher.accept(currentRobotPose);
            tagPosePublisher.accept(robotToAprilTagOffset);

            outputRobotSpeeds = holoController.calculate(
                currentRobotPose,
                robotToAprilTagOffset,
                0.0,
                Rotation2d.fromRadians(aprilTag.getBestCameraToTarget().getRotation().getZ())
                .plus(Rotation2d.fromRadians(robotToCamera.getRotation().getZ()))
                .times(-1)
                .minus(Rotation2d.fromDegrees(aprilTag.getYaw()))
            );

            robotVelocityVector = new Translation2d(
                -xController.calculate(currentRobotPose.getX(), 0.8),
                -yController.calculate(currentRobotPose.getY(), 0.0)
            );
            
            robotVelocityVector = robotVelocityVector.rotateBy(currentRobotPose.getRotation().times(-1)
                .plus(Rotation2d.fromDegrees(180))
            );

            SmartDashboard.putNumber("Holo Controller X Output", robotVelocityVector.getX());
            SmartDashboard.putNumber("Holo Controller Y Output", robotVelocityVector.getY());

            drivetrain.setControl(
                driveRequest
                    .withVelocityX(robotVelocityVector.getX())
                    .withVelocityY(robotVelocityVector.getY())
                    .withRotationalRate(outputRobotSpeeds.omegaRadiansPerSecond)
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (hasEndCondition) return holoController.atReference();
        return false;
    }
}
