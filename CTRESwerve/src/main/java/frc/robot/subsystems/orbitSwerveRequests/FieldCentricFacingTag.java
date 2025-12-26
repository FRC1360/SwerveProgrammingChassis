package frc.robot.subsystems.orbitSwerveRequests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.SwerveCamera;

public class FieldCentricFacingTag implements SwerveRequest {
    
    public double VelocityX = 0;
    public double VelocityY = 0;
    public double TargetRateFeedforward = 0;
    public double Deadband = 0;
    public double RotationalDeadband = 0;
    public double MaxAbsRotationalRate = 0;
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    public boolean DesaturateWheelSpeeds = true;
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);
    public SwerveCamera camera;
    public int tagID = 0;

    private final FieldCentric m_fieldCentric = new FieldCentric();

    public FieldCentricFacingTag() {
        HeadingController.enableContinuousInput(-180, 180);
    }

    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {

        double toApplyOmega = 0.0;
        boolean targetVisible = false;
        double targetYawDegrees = 0.0;

        List<PhotonPipelineResult> results = this.camera.getPipelineResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == this.tagID) {
                        targetYawDegrees = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        if (targetVisible) 
            toApplyOmega = HeadingController.calculate(targetYawDegrees, 0.0, parameters.timestamp);

        return m_fieldCentric
            .withVelocityX(VelocityX)
            .withVelocityY(VelocityY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
            .apply(parameters, modulesToApply);
    }
    
    public FieldCentricFacingTag withHeadingPID(double kP, double kI, double kD)
    {
        this.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricFacingTag withTargetTagID(int id) {
        this.tagID = id;
        return this;
    }

    public FieldCentricFacingTag withAppliedCamera(SwerveCamera camera) {
        this.camera = camera;
        return this;
    }
    
    public FieldCentricFacingTag withVelocityX(double newVelocityX) {
        this.VelocityX = newVelocityX;
        return this;
    }
    
    public FieldCentricFacingTag withVelocityX(LinearVelocity newVelocityX) {
        this.VelocityX = newVelocityX.in(MetersPerSecond);
        return this;
    }
    
    public FieldCentricFacingTag withVelocityY(double newVelocityY) {
        this.VelocityY = newVelocityY;
        return this;
    }
    
    public FieldCentricFacingTag withVelocityY(LinearVelocity newVelocityY) {
        this.VelocityY = newVelocityY.in(MetersPerSecond);
        return this;
    }
    
    public FieldCentricFacingTag withTargetRateFeedforward(double newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward;
        return this;
    }
    
    public FieldCentricFacingTag withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
        return this;
    }
    
    public FieldCentricFacingTag withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }
    
    public FieldCentricFacingTag withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }
    
    public FieldCentricFacingTag withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }
    
    public FieldCentricFacingTag withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }
    
    public FieldCentricFacingTag withMaxAbsRotationalRate(double newMaxAbsRotationalRate) {
        this.MaxAbsRotationalRate = newMaxAbsRotationalRate;
        return this;
    }
    
    public FieldCentricFacingTag withMaxAbsRotationalRate(AngularVelocity newMaxAbsRotationalRate) {
        this.MaxAbsRotationalRate = newMaxAbsRotationalRate.in(RadiansPerSecond);
        return this;
    }

    public FieldCentricFacingTag withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }
    
    public FieldCentricFacingTag withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }
    
    public FieldCentricFacingTag withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }
}
