package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveCamera;
import frc.robot.util.DriveConstants;

public class DriveCommands {

    public static Command aimAtTag(
        CommandSwerveDrivetrain drivetrain,
        SwerveCamera camera,
        int tagId,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier rotSupplier,
        boolean isFieldRelative
    ) {

    PIDController tagRPidController = 
        new PIDController(DriveConstants.tagRKp, DriveConstants.tagRKi, DriveConstants.tagRKd);

    return new FunctionalCommand(
        () -> {
            tagRPidController.setTolerance(DriveConstants.tagRTolerance);
        },
        () -> {
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
            if (targetVisible) {
                rotSpeed = tagRPidController.calculate(targetYawDegrees, 0.0);
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
        },
        interrupted -> {

        },
        () -> {
            // return tagRPidController.atSetpoint();
            return false;
        },
        drivetrain
    );

    }

}
