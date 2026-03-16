package frc.robot.subsystems;

import java.util.List;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public boolean autonMovingEnabled;
    public PathPlannerAuto a1;
    private Rotation2d lastKnownTagHeading;
    private Rotation2d originalHeading;
    private Rotation2d driverHeadingOffset;
    private boolean hasFieldPoseReference;
    private double lastVisionTimestampSeconds;
    private Pose2d lastAcceptedVisionPose;
    private boolean hasAcceptedVisionMeasurement;
    private String lastVisionRejectReason;
    private final PhotonVisionSubsystem photonVision;

    public Swerve(PhotonVisionSubsystem photonVision){
        this.photonVision = photonVision;
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.SwerveStartHeading);
    
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
            };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );
        autonMovingEnabled = true;

        lastKnownTagHeading = new Rotation2d(); 
        originalHeading = new Rotation2d();
        driverHeadingOffset = new Rotation2d();
        hasFieldPoseReference = false;
        lastVisionTimestampSeconds = -1.0;
        lastAcceptedVisionPose = new Pose2d();
        hasAcceptedVisionMeasurement = false;
        lastVisionRejectReason = "No vision measurements processed yet";
        }
                    
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(ChassisSpeeds speeds) {

        if (!autonMovingEnabled) {
            speeds = new ChassisSpeeds();
        }
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getDriverHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public TalonFX[] getTalons() {
        TalonFX[] talons = new TalonFX[8];
        for (int i = 0; i < 4; i++) {
            talons[i * 2] = (mSwerveMods[i].getTalons()[0]);
            talons[i * 2 + 1] = (mSwerveMods[i].getTalons()[1]);
        }
        return talons;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void enableAutonMoving() {
        autonMovingEnabled = true;
    }
    public void disableAutonMoving() {
        autonMovingEnabled = false;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometryAuto(Pose2d pose){
        return;
    }

    public void setPose(Pose2d pose) {
        hasFieldPoseReference = true;
        resetPoseTrackers(pose);
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public Rotation2d getDriverHeading() {
        return getGyroYaw().minus(driverHeadingOffset);
    }

    public boolean hasFieldPoseReference() {
        return hasFieldPoseReference;
    }

    public void setHeading(Rotation2d heading){
        resetPoseTrackers(new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        driverHeadingOffset = getGyroYaw();
    }

    public Command flipHeading(){
        return new InstantCommand(() -> resetPoseTrackers(
            new Pose2d(getPose().getTranslation(), getHeading().rotateBy(Rotation2d.fromDegrees(180)))
        ));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d(); //used to be: return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setOriginalHeading(Rotation2d heading) {
        originalHeading = heading;
    }

    private Pose2d getVisionSeedPose(PhotonVisionSubsystem.VisionMeasurement visionMeasurement) {
        Rotation2d seedHeading = getHeading();

        if (visionMeasurement.tagCount() > 1) {
            seedHeading = visionMeasurement.pose().getRotation();
        }

        return new Pose2d(visionMeasurement.pose().getTranslation(), seedHeading);
    }

    private void resetPoseTrackers(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        lastVisionTimestampSeconds = photonVision == null
            ? -1.0
            : photonVision.getBestEstimatedPose()
                .map(PhotonVisionSubsystem.VisionMeasurement::timestampSeconds)
                .orElse(-1.0);
    }

    private boolean shouldSeedPoseFromVision(PhotonVisionSubsystem.VisionMeasurement visionMeasurement) {
        if (hasFieldPoseReference || hasAcceptedVisionMeasurement) {
            return false;
        }

        return poseEstimator.getEstimatedPosition().getTranslation().getNorm()
                <= Constants.PhotonVisionConstants.poseSeedOriginToleranceMeters
            && swerveOdometry.getPoseMeters().getTranslation().getNorm()
                <= Constants.PhotonVisionConstants.poseSeedOriginToleranceMeters
            && visionMeasurement != null
            && visionMeasurement.tagCount() >= Constants.PhotonVisionConstants.minVisionSeedTagCount;
    }

    private boolean isPoseWithinField(Pose2d pose) {
        double fieldMargin = Constants.PhotonVisionConstants.visionFieldBoundaryMarginMeters;

        return pose.getX() >= -fieldMargin
            && pose.getX() <= Constants.FieldConstants.fieldLengthMeters + fieldMargin
            && pose.getY() >= -fieldMargin
            && pose.getY() <= Constants.FieldConstants.fieldWidthMeters + fieldMargin;
    }

    private String getVisionMeasurementRejectReason(PhotonVisionSubsystem.VisionMeasurement estimate) {
        if (estimate == null || estimate.tagCount() <= 0 || estimate.timestampSeconds() <= 0.0) {
            return "Missing tags or timestamp";
        }

        if (!Double.isFinite(estimate.pose().getX())
            || !Double.isFinite(estimate.pose().getY())
            || !Double.isFinite(estimate.pose().getRotation().getRadians())) {
            return "Pose contained NaN or infinity";
        }

        if (!isPoseWithinField(estimate.pose())) {
            return "Pose outside field bounds";
        }

        if (estimate.averageTagArea() < Constants.PhotonVisionConstants.minVisionTargetArea) {
            return "Target area too small";
        }

        if (estimate.tagCount() == 1) {
            if (estimate.averageTagDistanceMeters()
                > Constants.PhotonVisionConstants.maxSingleTagDistanceMeters) {
                return "Single-tag distance too large";
            }

            if (estimate.bestTargetAmbiguity() >= 0.0
                && estimate.bestTargetAmbiguity()
                    > Constants.PhotonVisionConstants.maxSingleTagAmbiguity) {
                return "Single-tag ambiguity too high";
            }
        } else if (estimate.averageTagDistanceMeters()
            > Constants.PhotonVisionConstants.maxMultiTagDistanceMeters) {
            return "Multi-tag distance too large";
        }

        if (shouldSeedPoseFromVision(estimate)) {
            return null;
        }

        if (!hasFieldPoseReference) {
            return "Waiting for multi-tag vision seed or manual pose reset";
        }

        double poseDeltaMeters = estimate.pose()
            .getTranslation()
            .getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        double maxPoseDeltaMeters = estimate.tagCount() > 1
            ? Constants.PhotonVisionConstants.maxMultiTagPoseDeltaMeters
            : Constants.PhotonVisionConstants.maxSingleTagPoseDeltaMeters;

        SmartDashboard.putNumber("Vision Pose Delta", poseDeltaMeters);
        SmartDashboard.putNumber("Vision Max Pose Delta", maxPoseDeltaMeters);

        if (poseDeltaMeters > maxPoseDeltaMeters) {
            return "Pose delta too large";
        }

        return null;
    }

    private double getVisionTranslationStdDev(PhotonVisionSubsystem.VisionMeasurement estimate) {
        double translationStdDev = Constants.PhotonVisionConstants.visionStdDevBase
            + (estimate.averageTagDistanceMeters()
                * Constants.PhotonVisionConstants.visionStdDevPerMeter
                / Math.max(estimate.tagCount(), 1));

        if (estimate.tagCount() == 1) {
            translationStdDev *= Constants.PhotonVisionConstants.singleTagStdDevMultiplier;
        }

        if (estimate.averageTagArea() < 0.15) {
            translationStdDev *= Constants.PhotonVisionConstants.lowAreaStdDevMultiplier;
        }

        return Math.max(0.05, Math.min(translationStdDev, 2.0));
    }

    private void addVisionMeasurementsIfAvailable() {
        if (photonVision == null) {
            SmartDashboard.putBoolean("Vision Measurement Accepted", false);
            SmartDashboard.putBoolean("Vision Measurement Accepted This Cycle", false);
            SmartDashboard.putString("Vision Reject Reason", "PhotonVision subsystem missing");
            return;
        }

        List<PhotonVisionSubsystem.VisionMeasurement> visionMeasurements =
            photonVision.getVisionMeasurementsSince(lastVisionTimestampSeconds);
        boolean acceptedMeasurementThisCycle = false;
        boolean seededPoseThisCycle = false;
        String rejectReasonThisCycle = visionMeasurements.isEmpty()
            ? photonVision.getStatusSummary()
            : "No valid vision measurements this cycle";

        for (PhotonVisionSubsystem.VisionMeasurement visionMeasurement : visionMeasurements) {
            String rejectReason = getVisionMeasurementRejectReason(visionMeasurement);
            if (rejectReason != null) {
                rejectReasonThisCycle = rejectReason;
                continue;
            }

            double translationStdDev = getVisionTranslationStdDev(visionMeasurement);
            if (shouldSeedPoseFromVision(visionMeasurement)) {
                hasFieldPoseReference = true;
                resetPoseTrackers(getVisionSeedPose(visionMeasurement));
                seededPoseThisCycle = true;
            } else {
                poseEstimator.addVisionMeasurement(
                    visionMeasurement.pose(),
                    visionMeasurement.timestampSeconds(),
                    VecBuilder.fill(
                        translationStdDev,
                        translationStdDev,
                        Constants.PhotonVisionConstants.visionRotationStdDev
                    )
                );
            }

            lastVisionTimestampSeconds = visionMeasurement.timestampSeconds();
            lastAcceptedVisionPose = visionMeasurement.pose();
            hasAcceptedVisionMeasurement = true;
            hasFieldPoseReference = true;
            acceptedMeasurementThisCycle = true;
            lastVisionRejectReason = "Accepted";

            SmartDashboard.putNumber("Vision Tag Count", visionMeasurement.tagCount());
            SmartDashboard.putNumber(
                "Vision Avg Tag Dist",
                visionMeasurement.averageTagDistanceMeters()
            );
            SmartDashboard.putNumber("Vision Avg Tag Area", visionMeasurement.averageTagArea());
            SmartDashboard.putNumber("Vision Std Dev XY", translationStdDev);
            SmartDashboard.putNumber("Vision Pose X", visionMeasurement.pose().getX());
            SmartDashboard.putNumber("Vision Pose Y", visionMeasurement.pose().getY());
            SmartDashboard.putString("Vision Camera", visionMeasurement.cameraName());
        }

        if (!acceptedMeasurementThisCycle) {
            lastVisionRejectReason = rejectReasonThisCycle;
        }

        SmartDashboard.putBoolean("Vision Measurement Accepted", hasAcceptedVisionMeasurement);
        SmartDashboard.putBoolean("Vision Measurement Accepted This Cycle", acceptedMeasurementThisCycle);
        SmartDashboard.putBoolean("Vision Pose Seeded This Cycle", seededPoseThisCycle);
        SmartDashboard.putString("Vision Reject Reason", lastVisionRejectReason);
    }

    public void updateParallelMotion(boolean parallelModeActive,
                                 boolean returnToOriginal,
                                 boolean allowRotation,
                                 PhotonVisionSubsystem photonVision) {

        Pose2d detectedPose = photonVision == null
            ? null
            : photonVision.getBestEstimatedPose()
                .map(PhotonVisionSubsystem.VisionMeasurement::pose)
                .orElse(null);

        // 1) If we see a new valid pose, update lastKnownTagHeading
        //    (only do this if we actually got a detection!)
        if (detectedPose != null) {
            lastKnownTagHeading = detectedPose.getRotation();
        }

        // 2) If the driver is holding LB (parallelModeActive),
        //    and not holding RB (which would allow rotation),
        //    then forcibly lock heading to the last known tag heading (if we have one).
        if (parallelModeActive && !allowRotation && lastKnownTagHeading != null) {
            setHeading(lastKnownTagHeading);
        }

        // 3) If LB has just been released, revert to the stored original heading
        if (returnToOriginal) {
            setHeading(originalHeading);
        }
    }




    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getGyroYaw(), getModulePositions());
        addVisionMeasurementsIfAvailable();

        SmartDashboard.putNumber("Odometry X", getOdometryPose().getX());
        SmartDashboard.putNumber("Odometry Y", getOdometryPose().getY());
        SmartDashboard.putNumber("Estimated Pose X", getPose().getX());
        SmartDashboard.putNumber("Estimated Pose Y", getPose().getY());
        SmartDashboard.putNumber("Estimated Pose Heading", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Driver Heading", getDriverHeading().getDegrees());
        SmartDashboard.putBoolean("Field Pose Ready", hasFieldPoseReference);
        SmartDashboard.putNumber("Last Vision Pose X", lastAcceptedVisionPose.getX());
        SmartDashboard.putNumber("Last Vision Pose Y", lastAcceptedVisionPose.getY());

        for(SwerveModule mod : mSwerveMods){
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    

            // SmartDashboard.putNumber("Pigeon ang vel", gyro.getAngularVelocityXDevice().getValueAsDouble());
            SmartDashboard.putNumber("Pigeon Yaw", gyro.getYaw().getValueAsDouble());  

                
        }
    }
}       


/*
 * FEIN FEIN FEIN FEIN     FEIN FEIN FEIN FEIN      FEIN FEIN FEIN FEIN FEIN       FEIN FEIN            FEIN
 * FEIN                    FEIN                               FEIN                 FEIN   FEIN          FEIN
 * FEIN                    FEIN                               FEIN                 FEIN      FEIN       FEIN
 * FEIN FEIN FEIN          FEIN FEIN FEIN                     FEIN                 FEIN         FEIN    FEIN
 * FEIN                    FEIN                               FEIN                 FEIN            FEIN FEIN
 * FEIN                    FEIN                               FEIN                 FEIN               FEIN
 * FEIN                    FEIN FEIN FEIN FEIN      FEIN FEIN FEIN FEIN FEIN       FEIN                 FEIN
 */


 /* */ 
