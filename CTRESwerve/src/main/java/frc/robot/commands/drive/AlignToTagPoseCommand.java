// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveCamera;
import frc.robot.util.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagPoseCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveCamera camera;
    private final int tagId;
    private final boolean hasEndCondition;

    private final HolonomicDriveController holoController;
    private final SwerveRequest.RobotCentric driveRequest;
    private final Pose2d robotToAprilTagOffset;

    /*
     * This variable determines whether the algorithm will directly use the aprilTag's yaw or the pose estimation yaw
     */
    private final double methodThreshold = 0.7;
    private double methodBias;
    
    private final StructPublisher<Pose2d> tagPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/TagPose", Pose2d.struct)
            .publish();
    
    private final StructPublisher<Pose2d> robotPosePublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("Holonomic Controller/RobotPose", Pose2d.struct)
            .publish();

    /** Creates a new aimAtTag. */
    public AlignToTagPoseCommand(
        CommandSwerveDrivetrain drivetrain,
        SwerveCamera camera,
        int tagId,
        boolean hasEndCondition,
        Pose2d robotToAprilTagOffset
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.tagId = tagId;
        this.hasEndCondition = hasEndCondition;
        this.robotToAprilTagOffset = robotToAprilTagOffset;

        this.holoController = new HolonomicDriveController(
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new ProfiledPIDController(
                5.0, 0.0, 0.0,
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
        Pose2d currentRobotPose = new Pose2d();
        Pose2d fieldRelativeAprilTagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagId).get().toPose2d();
        ChassisSpeeds outputRobotSpeeds = new ChassisSpeeds();
        Transform3d robotToCamera = camera.getRobotToCamera();
        Rotation2d targetHeading = new Rotation2d();

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

        calculateMethodBias(targetVisible);
        SmartDashboard.putNumber("Commands/" + getName() + "/targetVisible", targetVisible ? 1.0 : 0.0);
        SmartDashboard.putNumber("Commands/" + getName() + "/methodBias", methodBias);

        if ((methodBias > methodThreshold) && targetVisible) {
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
            targetHeading = 
                Rotation2d.fromRadians(aprilTag.getBestCameraToTarget().getRotation().getZ())
                .plus(Rotation2d.fromRadians(robotToCamera.getRotation().getZ()))
                .times(-1)
                .minus(Rotation2d.fromDegrees(aprilTag.getYaw()));    

            outputRobotSpeeds = holoController.calculate(
                currentRobotPose,
                robotToAprilTagOffset,
                0.0,
                targetHeading
            );
        } else if ((methodBias > methodThreshold) && !targetVisible) {
            currentRobotPose = new Pose2d();
            targetHeading = new Rotation2d();
            outputRobotSpeeds = new ChassisSpeeds();
        } else if (methodBias < methodThreshold) {
            currentRobotPose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get().relativeTo(fieldRelativeAprilTagPose);
            currentRobotPose = currentRobotPose.rotateBy(fieldRelativeAprilTagPose.getRotation());
            targetHeading = Rotation2d.fromDegrees(180).minus(Rotation2d.fromRadians(robotToCamera.getRotation().getZ()));

            outputRobotSpeeds = holoController.calculate(
                currentRobotPose, 
                robotToAprilTagOffset, 
                0.0, 
                targetHeading
            );
        }
            
        robotPosePublisher.accept(currentRobotPose);
        tagPosePublisher.accept(robotToAprilTagOffset);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(outputRobotSpeeds.vxMetersPerSecond)
                .withVelocityY(outputRobotSpeeds.vyMetersPerSecond)
                .withRotationalRate(outputRobotSpeeds.omegaRadiansPerSecond)
        );
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

    // Implementation of the method determining algorithm
    private void calculateMethodBias(boolean isTargetVisible) {
        if (isTargetVisible) methodBias = methodBias + 0.5;
        else methodBias = methodBias - 0.1;

        methodBias = MathUtil.clamp(methodBias, 0.0, 1.0);
    }
}
