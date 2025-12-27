// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveCommands;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
public class AlignToTagCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveCamera camera;
    private final int tagId;
    private final boolean hasEndCondition;

    private final HolonomicDriveController holoController;
    private final SwerveRequest.RobotCentric driveRequest;
    private final Pose2d tagToRobotOffset;
    
    private final StructPublisher<Pose2d> tagPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/TagPose", Pose2d.struct)
            .publish();
    
    private final StructPublisher<Pose2d> robotPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/RobotPose", Pose2d.struct)
            .publish();

    /** Creates a new aimAtTag. */
    public AlignToTagCommand(
        CommandSwerveDrivetrain drivetrain,
        SwerveCamera camera,
        int tagId,
        boolean hasEndCondition,
        Pose2d tagToRobotOffset
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.tagId = tagId;
        this.hasEndCondition = hasEndCondition;
        this.tagToRobotOffset = tagToRobotOffset;

        this.holoController = new HolonomicDriveController(
            new PIDController(4.0, 0.0, 0.0),
            new PIDController(4.0, 0.0, 0.0),
            new ProfiledPIDController(
                3.0, 0.0, 0.1,
                new TrapezoidProfile.Constraints(DriveConstants.MaxAngularSpeedRadians, DriveConstants.MaxAngularAccelerationRadians)
            )
        );

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
        Pose2d currentTagPose = new Pose2d();
        ChassisSpeeds outputRobotSpeeds;

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
            SmartDashboard.putNumber("Holonomic Controller/April Tag Theta", 
                Units.radiansToDegrees(aprilTag.getBestCameraToTarget().getRotation().getZ())
            );
            SmartDashboard.putNumber("Holonomic Controller/April Tag Yaw", 
                aprilTag.getYaw()
            );
            SmartDashboard.putNumber("Holonomic Controller/April Tag dx", 
                aprilTag.getBestCameraToTarget().getX()
            );
            SmartDashboard.putNumber("Holonomic Controller/April Tag dy", 
                aprilTag.getBestCameraToTarget().getY()
            );

            SmartDashboard.putNumber("Holonomic Controller/Target Heading", 
                Rotation2d.fromRadians(aprilTag.getBestCameraToTarget().getRotation().getZ())
                .plus(Rotation2d.fromRadians(camera.getRobotToCamera().getRotation().getZ()))
                .times(-1)
                .minus(Rotation2d.fromDegrees(aprilTag.getYaw()))
                .getDegrees()
            );
            
            currentTagPose = new Pose2d(
                aprilTag.getBestCameraToTarget().getX(),
                aprilTag.getBestCameraToTarget().getY(),
                aprilTag.getBestCameraToTarget().getRotation().toRotation2d()
            );
            currentTagPose = currentTagPose.rotateBy(
                Rotation2d.fromRadians(camera.getRobotToCamera().getRotation().getZ())
            );

            outputRobotSpeeds =
                holoController.calculate(
                    tagToRobotOffset,
                    currentTagPose,
                    0.0,
                    Rotation2d.fromDegrees(aprilTag.getYaw())
                        .minus(currentTagPose.getRotation())
                        .plus(Rotation2d.fromDegrees(180))
                        .plus(Rotation2d.fromRadians(camera.getRobotToCamera().getRotation().getZ()))
                );

            tagPosePublisher.accept(currentTagPose);
            robotPosePublisher.accept(tagToRobotOffset);

            if (!holoController.atReference())
                drivetrain.setControl(
                    driveRequest
                        .withVelocityX(outputRobotSpeeds.vxMetersPerSecond)
                        .withVelocityY(outputRobotSpeeds.vyMetersPerSecond)
                        .withRotationalRate(-outputRobotSpeeds.omegaRadiansPerSecond)
                );
            else 
                drivetrain.setControl(
                    driveRequest
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.0)
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
