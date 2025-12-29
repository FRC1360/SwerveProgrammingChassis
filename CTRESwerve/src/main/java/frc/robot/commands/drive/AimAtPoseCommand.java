// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtPoseCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d poseToAimAt;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final boolean isFieldRelative;
    private final boolean hasEndCondition;

    private final PIDController tagRPidController;

    /** Creates a new aimAtTag. */
    public AimAtPoseCommand(
        CommandSwerveDrivetrain drivetrain,
        Pose2d poseToAimAt,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        boolean isFieldRelative,
        boolean hasEndCondition
    ) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = drivetrain;
        this.poseToAimAt = poseToAimAt;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.isFieldRelative = isFieldRelative;
        this.hasEndCondition = hasEndCondition;

        this.tagRPidController = new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);

        addRequirements(this.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        tagRPidController.reset();
        tagRPidController.setTolerance(DriveConstants.tagRTolerance);
        tagRPidController.enableContinuousInput(-180, 180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xSpeed, ySpeed, rotSpeed = 0.0;
        Pose2d robotPose = drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get();
        double deltaHeading = PhotonUtils.getYawToPose(robotPose, poseToAimAt).getDegrees();

        xSpeed = xSupplier.getAsDouble();
        ySpeed = ySupplier.getAsDouble();
        rotSpeed = -tagRPidController.calculate(deltaHeading, 0.0);

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
}
