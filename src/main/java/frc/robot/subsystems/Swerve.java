package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

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
import edu.wpi.first.wpilibj.DriverStation;
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
    private double lastVisionTimestampSeconds;
    private Pose2d lastAcceptedVisionPose;

    public Swerve(){
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
        lastVisionTimestampSeconds = -1.0;
        lastAcceptedVisionPose = new Pose2d();
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
                                    getHeading()
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
        resetPoseTrackers(pose);
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        resetPoseTrackers(new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        resetPoseTrackers(new Pose2d(getPose().getTranslation(), new Rotation2d()));
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

    private void resetPoseTrackers(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        lastVisionTimestampSeconds = -1.0;
    }

    private PoseEstimate getAlliancePoseEstimate(boolean useMegaTag2) {
        String limelightName = Constants.LimelightConstants.limelightName;

        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                getGyroYaw().getDegrees(),
                0,
                0,
                0,
                0,
                0
            );
        }

        boolean isRedAlliance =
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;

        if (isRedAlliance) {
            return useMegaTag2
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
        }

        return useMegaTag2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    private PoseEstimate getPreferredVisionMeasurement() {
        PoseEstimate megaTag1Estimate = getAlliancePoseEstimate(false);
        PoseEstimate megaTag2Estimate = getAlliancePoseEstimate(true);

        PoseEstimate preferredEstimate = megaTag2Estimate.tagCount >= 2 ? megaTag2Estimate : megaTag1Estimate;
        PoseEstimate fallbackEstimate = preferredEstimate == megaTag1Estimate ? megaTag2Estimate : megaTag1Estimate;

        if (isVisionMeasurementValid(preferredEstimate)) {
            return preferredEstimate;
        }

        if (isVisionMeasurementValid(fallbackEstimate)) {
            return fallbackEstimate;
        }

        return null;
    }

    private boolean isVisionMeasurementValid(PoseEstimate estimate) {
        if (estimate == null || estimate.tagCount <= 0 || estimate.timestampSeconds <= 0.0) {
            return false;
        }

        if (estimate.timestampSeconds <= lastVisionTimestampSeconds) {
            return false;
        }

        if (!Double.isFinite(estimate.pose.getX())
            || !Double.isFinite(estimate.pose.getY())
            || !Double.isFinite(estimate.pose.getRotation().getRadians())) {
            return false;
        }

        if (estimate.avgTagArea < Constants.LimelightConstants.minVisionTagArea) {
            return false;
        }

        if (estimate.tagCount == 1) {
            if (estimate.avgTagDist > Constants.LimelightConstants.maxSingleTagDistanceMeters) {
                return false;
            }

            if (estimate.rawFiducials.length > 0
                && estimate.rawFiducials[0].ambiguity > Constants.LimelightConstants.maxSingleTagAmbiguity) {
                return false;
            }
        } else if (estimate.avgTagDist > Constants.LimelightConstants.maxMultiTagDistanceMeters) {
            return false;
        }

        double poseDeltaMeters = estimate.pose
            .getTranslation()
            .getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        double maxPoseDeltaMeters = estimate.tagCount > 1
            ? Constants.LimelightConstants.maxMultiTagPoseDeltaMeters
            : Constants.LimelightConstants.maxSingleTagPoseDeltaMeters;

        return poseDeltaMeters <= maxPoseDeltaMeters;
    }

    private double getVisionTranslationStdDev(PoseEstimate estimate) {
        double translationStdDev = Constants.LimelightConstants.visionStdDevBase
            + (estimate.avgTagDist * Constants.LimelightConstants.visionStdDevPerMeter
                / Math.max(estimate.tagCount, 1));

        if (estimate.tagCount == 1) {
            translationStdDev *= Constants.LimelightConstants.singleTagStdDevMultiplier;
        }

        if (estimate.avgTagArea < 0.15) {
            translationStdDev *= Constants.LimelightConstants.lowAreaStdDevMultiplier;
        }

        return Math.max(0.05, Math.min(translationStdDev, 2.0));
    }

    private void addVisionMeasurementIfAvailable() {
        PoseEstimate visionMeasurement = getPreferredVisionMeasurement();

        SmartDashboard.putBoolean("Vision Measurement Accepted", visionMeasurement != null);

        if (visionMeasurement == null) {
            return;
        }

        double translationStdDev = getVisionTranslationStdDev(visionMeasurement);
        poseEstimator.addVisionMeasurement(
            visionMeasurement.pose,
            visionMeasurement.timestampSeconds,
            VecBuilder.fill(
                translationStdDev,
                translationStdDev,
                Constants.LimelightConstants.visionRotationStdDev
            )
        );

        lastVisionTimestampSeconds = visionMeasurement.timestampSeconds;
        lastAcceptedVisionPose = visionMeasurement.pose;

        SmartDashboard.putNumber("Vision Tag Count", visionMeasurement.tagCount);
        SmartDashboard.putNumber("Vision Avg Tag Dist", visionMeasurement.avgTagDist);
        SmartDashboard.putNumber("Vision Avg Tag Area", visionMeasurement.avgTagArea);
        SmartDashboard.putNumber("Vision Std Dev XY", translationStdDev);
        SmartDashboard.putNumber("Vision Pose X", visionMeasurement.pose.getX());
        SmartDashboard.putNumber("Vision Pose Y", visionMeasurement.pose.getY());
    }

    public void updateParallelMotion(boolean parallelModeActive,
                                 boolean returnToOriginal,
                                 boolean allowRotation,
                                 Limelight limelight) {

        // Attempt to get a fresh detected Pose from the limelight
        Pose2d detectedPose = limelight.getAdjustedRobotPose();

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
        addVisionMeasurementIfAvailable();

        SmartDashboard.putNumber("Odometry X", getOdometryPose().getX());
        SmartDashboard.putNumber("Odometry Y", getOdometryPose().getY());
        SmartDashboard.putNumber("Estimated Pose X", getPose().getX());
        SmartDashboard.putNumber("Estimated Pose Y", getPose().getY());
        SmartDashboard.putNumber("Estimated Pose Heading", getPose().getRotation().getDegrees());
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
