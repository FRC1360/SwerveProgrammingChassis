// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveCamera;
import frc.robot.util.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtTagPoseCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveCamera camera;
    private final int tagId;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final boolean isFieldRelative;
    private final boolean hasEndCondition;

    /*
     * This variable determines whether the algorithm will directly use the aprilTag's yaw or the pose estimation yaw
     */
    private final double methodThreshold = 0.7;
    private double methodBias;

    private final PIDController tagRPidController;

    /** Creates a new aimAtTag. */
    public AimAtTagPoseCommand(
        CommandSwerveDrivetrain drivetrain,
        SwerveCamera camera,
        int tagId,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        boolean isFieldRelative,
        boolean hasEndCondition
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.tagId = tagId;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.isFieldRelative = isFieldRelative;
        this.hasEndCondition = hasEndCondition;

        this.methodBias = this.methodThreshold;

        this.tagRPidController = new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);

        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        tagRPidController.reset();
        tagRPidController.setTolerance(DriveConstants.tagRTolerance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed, ySpeed, rotSpeed = 0.0;
        boolean targetVisible = false;
        double targetYawDegrees = 0.0;

        List<PhotonPipelineResult> results = camera.getPipelineResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == tagId) {
                        targetYawDegrees = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        xSpeed = xSupplier.getAsDouble();
        ySpeed = ySupplier.getAsDouble();
        calculateMethodBias(targetVisible);
        SmartDashboard.putNumber("Commands/" + getName() + "/targetVisible", targetVisible ? 1.0 : 0.0);
        SmartDashboard.putNumber("Commands/" + getName() + "/methodBias", methodBias);

        if ((methodBias > methodThreshold) && targetVisible) {
            rotSpeed = tagRPidController.calculate(targetYawDegrees, 0.0);
        } else if ((methodBias > methodThreshold) && !targetVisible) {
            rotSpeed = 0.0;
        } else if (methodBias < methodThreshold) {
            rotSpeed = -tagRPidController.calculate(
                PhotonUtils.getYawToPose(
                    drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get(),
                    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagId).get().toPose2d()
                ).getDegrees(),
                Units.radiansToDegrees(camera.getRobotToCamera().getRotation().getZ())
            );
        }

        // exclude rotational deadbands as controlling rot thru PID
        if (isFieldRelative) {
            drivetrain.setControl(
                new SwerveRequest.FieldCentric()
                    .withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed)
                    .withDeadband(DriveConstants.MaxSpeed * DriveConstants.joystickDeadbandDecimal)
            );
        } else {
            drivetrain.setControl(
                new SwerveRequest.RobotCentric()
                    .withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed)
                    .withDeadband(DriveConstants.MaxSpeed * DriveConstants.joystickDeadbandDecimal)
            );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (hasEndCondition) return tagRPidController.atSetpoint();
        return false;
    }

    // Implementation of the method determining algorithm
    private void calculateMethodBias(boolean isTargetVisible) {
        if (isTargetVisible) methodBias = methodBias + 0.5;
        else methodBias = methodBias - 0.1;

        methodBias = MathUtil.clamp(methodBias, 0.0, 1.0);
    }
}
