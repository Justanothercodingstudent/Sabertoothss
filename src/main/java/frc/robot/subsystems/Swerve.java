package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.BooleanSubscriber;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
    private final Vision vision;
    public SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public boolean autonMovingEnabled;

    private boolean Hello;
    
    private Rotation2d lastKnownTagHeading;
    private Rotation2d originalHeading;
    private Rotation2d driverForwardHeading;
    private double lastVisionTimestampSeconds;
    private Pose2d lastAcceptedVisionPose;
    private int currentLimelightImuMode;
    private Pose2d lastRawVisionPose;
    private double lastRawVisionTimestampSeconds;
    private final Map<String, Double> lastVisionTimestampsByCamera = new HashMap<>();
    private final Map<String, Pose2d> lastAcceptedVisionPosesByCamera = new HashMap<>();
    private final Map<String, Pose2d> lastRawVisionPosesByCamera = new HashMap<>();
    private final Map<String, Double> lastRawVisionTimestampsByCamera = new HashMap<>();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> visionLog = driveStateTable.getStructTopic("latestPoseEstimate", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> lastVisionLog = driveStateTable.getStructTopic("lastAcceptedVisionPosesByCamera", Pose2d.struct).publish();

    public Swerve(Vision vision){
        this.vision = vision == null ? new Vision() : vision;
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.CTRE.CANIVORE_NAME);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(Constants.Swerve.SwerveStartHeading);
    
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
            };

        Rotation2d startupHeading = getGyroYaw();
        Pose2d startupPose = new Pose2d(new Translation2d(), startupHeading);

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            startupHeading,
            getModulePositions(),
            startupPose
        );
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            startupHeading,
            getModulePositions(),
            startupPose
        );
    
        SmartDashboard.putData("Field", field);
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        autonMovingEnabled = true;
        configureAutoBuilder();

        lastKnownTagHeading = new Rotation2d(); 
        originalHeading = startupHeading;
        driverForwardHeading = startupHeading.rotateBy(Rotation2d.fromDegrees(180));
        lastVisionTimestampSeconds = -1.0;
        lastAcceptedVisionPose = new Pose2d();
        currentLimelightImuMode = -1;
        lastRawVisionPose = new Pose2d();
        lastRawVisionTimestampSeconds = -1.0;
        initializeVisionState();
        applyLimelightImuMode(Constants.LimelightConstants.limelightImuSeedMode);
        }

    private void initializeVisionState() {
        for (Limelight limelight : vision.getLimelights()) {
            String cameraName = limelight.getName();
            lastVisionTimestampsByCamera.put(cameraName, -1.0);
            lastAcceptedVisionPosesByCamera.put(cameraName, new Pose2d());
            lastRawVisionPosesByCamera.put(cameraName, new Pose2d());
            lastRawVisionTimestampsByCamera.put(cameraName, -1.0);
        }
    }

    private void configureAutoBuilder() {
        RobotConfig robotConfig = Constants.PATHPLANNER_ROBOT_CONFIG;
        if (robotConfig == null) {
            DriverStation.reportError("PathPlanner AutoBuilder was not configured because the robot config failed to load.", false);
            return;
        }

        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards),
            new PPHolonomicDriveController(
                new PIDConstants(
                    Constants.AutoConstants.translationKP,
                    Constants.AutoConstants.translationKI,
                    Constants.AutoConstants.translationKD
                ),
                new PIDConstants(
                    Constants.AutoConstants.rotationKP,
                    Constants.AutoConstants.rotationKI,
                    Constants.AutoConstants.rotationKD
                )
            ),
            robotConfig,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
            this
        );
    }

    public void AutoAim(double[] tagData, SlewRateLimiter tagAimOutputLimiter, PIDController tagAimController, double speed){
        double yawErrorDegrees = tagData[1];
            double autoRotationCommand = tagAimOutputLimiter.calculate(MathUtil.clamp(
                tagAimController.calculate(yawErrorDegrees, 0.0),
                -speed,
                speed
            ));

            if (tagAimController.atSetpoint()) {
                autoRotationCommand = 0.0;
                tagAimOutputLimiter.reset(0.0);
            }
    } //I dont know if this'll work cuz im a chud
                    
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(ChassisSpeeds speeds) {
        driveRobotRelative(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        if (!autonMovingEnabled) {
            speeds = new ChassisSpeeds();
        }

        setChassisSpeeds(speeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        driveRobotRelative(speeds);
    }

    private void SetX(){
        mSwerveMods[0].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), true);
        mSwerveMods[1].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), true);
        mSwerveMods[2].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), true);
        mSwerveMods[3].setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), true);
    }

    private void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds =
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getHeading()
                            )
                            : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation
                            );

        if (!autonMovingEnabled && !isOpenLoop) {
            chassisSpeeds = new ChassisSpeeds();
        }

        boolean SoManyVariables = Robot.XToggle;

        if (chassisSpeeds.vxMetersPerSecond == 0.0 && chassisSpeeds.vyMetersPerSecond == 0.0 && chassisSpeeds.omegaRadiansPerSecond == 0.0 && SoManyVariables) {
            Hello = true;
            SetX();
            return;
        } else {
            Hello = false;
        }

        setChassisSpeeds(chassisSpeeds, isOpenLoop);
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

    public void seedFieldHeading(Rotation2d fieldHeading) {
        if (!Double.isFinite(fieldHeading.getRadians())) {
            return;
        }

        gyro.setYaw(fieldHeading.getDegrees());
        resetPoseTrackers(new Pose2d(getPose().getTranslation(), fieldHeading));

        if (DriverStation.isDisabled()) {
            applyLimelightImuMode(Constants.LimelightConstants.limelightImuSeedMode);
        }

        pushFieldHeadingToLimelight(fieldHeading);
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

    public void captureDriverForwardHeadingFromCurrentFieldHeading() {
        driverForwardHeading = getGyroYaw().rotateBy(Rotation2d.fromDegrees(180));
    }

    public Rotation2d getDriverForwardHeading() {
        return driverForwardHeading;
    }

    private void resetPoseTrackers(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        lastVisionTimestampSeconds = -1.0;
        for (String cameraName : lastVisionTimestampsByCamera.keySet()) {
            lastVisionTimestampsByCamera.put(cameraName, -1.0);
        }
    }

    private Vision.CameraPoseEstimate[] getMegaTag2PoseEstimates() {
        pushFieldHeadingToLimelight(getGyroYaw());
        return vision.getMegaTag2PoseEstimates();
    }

    private void applyLimelightImuMode(int imuMode) {
        if (currentLimelightImuMode == imuMode) {
            return;
        }

        vision.setIMUMode(imuMode);
        currentLimelightImuMode = imuMode;
    }

    private void pushFieldHeadingToLimelight(Rotation2d fieldHeading) {
        if (!Double.isFinite(fieldHeading.getRadians())) {
            return;
        }

        vision.pushFieldHeadingToLimelights(fieldHeading);
    }

    private void updateLimelightImuMode() {
        int targetImuMode = DriverStation.isDisabled()
            ? Constants.LimelightConstants.limelightImuSeedMode
            : Constants.LimelightConstants.limelightImuEnabledMode;
        applyLimelightImuMode(targetImuMode);
    }

    private List<Vision.CameraPoseEstimate> getMegaTag2VisionMeasurements() {
        List<Vision.CameraPoseEstimate> validMeasurements = new ArrayList<>();

        for (Vision.CameraPoseEstimate cameraEstimate : getMegaTag2PoseEstimates()) {
            PoseEstimate poseEstimate = cameraEstimate.poseEstimate;
            if (poseEstimate != null) {
                String cameraName = cameraEstimate.limelight.getName();
                lastRawVisionPose = poseEstimate.pose;
                lastRawVisionTimestampSeconds = poseEstimate.timestampSeconds;
                lastRawVisionPosesByCamera.put(cameraName, poseEstimate.pose);
                lastRawVisionTimestampsByCamera.put(cameraName, poseEstimate.timestampSeconds);
            }

            if (isVisionMeasurementValid(cameraEstimate)) {
                validMeasurements.add(cameraEstimate);
            }
        }

        // validMeasurements.sort(
        //     Comparator.comparingDouble((Vision.CameraPoseEstimate estimate) -> estimate.poseEstimate.timestampSeconds)
        //         .thenComparingDouble(estimate -> getVisionTranslationStdDev(estimate.poseEstimate))
        // );

        return validMeasurements;
    }

    private boolean isVisionMeasurementValid(Vision.CameraPoseEstimate cameraEstimate) {
        PoseEstimate estimate = cameraEstimate.poseEstimate;
        if (estimate == null || estimate.tagCount <= 0 || estimate.timestampSeconds <= 0.0) {
            return false;
        }

        double lastCameraTimestamp = lastVisionTimestampsByCamera.getOrDefault(
            cameraEstimate.limelight.getName(),
            -1.0
        );
        
        if (estimate.timestampSeconds <= lastCameraTimestamp) {
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

    private boolean configureVisionMeasurementStdDevs(PoseEstimate estimate) {
        double xyStds;
        double radStds;
        if (estimate.tagCount > 1) {
            if (Robot.gameMode == GameMode.TELEOP) {
                // In teleop, trust multi-tag solves more aggressively.
                if (estimate.tagCount > 2) {
                    xyStds = Math.hypot(0.002, 0.003);
                } else {
                    xyStds = Math.hypot(0.005, 0.008);
                }
            } else {
                xyStds = Math.hypot(0.014, 0.016);
            }
            radStds = Units.degreesToRadians(2);
        } else if (estimate.avgTagArea > 0.14) {
            xyStds = Math.hypot(0.015, 0.033);
            radStds = Units.degreesToRadians(7);
        } else {
            return false;
        }

        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, radStds));
        return true;
    }

    

    private void addVisionMeasurementIfAvailable() {
                
        List<Vision.CameraPoseEstimate> visionMeasurements = getMegaTag2VisionMeasurements();

        

        // SmartDashboard.putBoolean("Vision Measurement Accepted", !visionMeasurements.isEmpty());
        // SmartDashboard.putNumber("Vision Accepted Measurement Count", visionMeasurements.size());

        if (visionMeasurements.isEmpty()) {
            // SmartDashboard.putBoolean("Vision Measurement Invalid - No Valid Estimate", true);
            return;
        }

        // SmartDashboard.putBoolean("Vision Measurement Invalid - No Valid Estimate", false);

        Vision.CameraPoseEstimate latestMeasurement = null;

        for (Vision.CameraPoseEstimate cameraMeasurement : visionMeasurements) {
            PoseEstimate visionMeasurement = cameraMeasurement.poseEstimate;
            if (!configureVisionMeasurementStdDevs(visionMeasurement)) {
                continue;
            }

            Pose2d fieldOrientedVisionPose = visionMeasurement.pose;
            poseEstimator.addVisionMeasurement(
                fieldOrientedVisionPose,
                visionMeasurement.timestampSeconds
            );

            String cameraName = cameraMeasurement.limelight.getName();
            lastVisionTimestampsByCamera.put(cameraName, visionMeasurement.timestampSeconds);
            lastAcceptedVisionPosesByCamera.put(cameraName, fieldOrientedVisionPose);
            lastVisionTimestampSeconds = Math.max(lastVisionTimestampSeconds, visionMeasurement.timestampSeconds);
            lastAcceptedVisionPose = fieldOrientedVisionPose;
            latestMeasurement = cameraMeasurement;
        }

        if (latestMeasurement == null) {
            return;
        }

        Pose2d latestAcceptedPose = lastAcceptedVisionPose;
        // SmartDashboard.putString("Vision Selected Camera", latestMeasurement.limelight.getName());
        // SmartDashboard.putNumber("Vision Tag Count", latestMeasurement.poseEstimate.tagCount);
        // SmartDashboard.putNumber("Vision Avg Tag Dist", latestMeasurement.poseEstimate.avgTagDist);
        // SmartDashboard.putNumber("Vision Avg Tag Area", latestMeasurement.poseEstimate.avgTagArea);
        SmartDashboard.putNumber("Vision Pose X", latestAcceptedPose.getX());
        SmartDashboard.putNumber("Vision Pose Y", latestAcceptedPose.getY());
        SmartDashboard.putNumber("Vision Pose Heading", latestAcceptedPose.getRotation().getDegrees());
        
    }

    public Pose2d visionPose(){
        List<Vision.CameraPoseEstimate> visionMeasurements = getMegaTag2VisionMeasurements();

        Vision.CameraPoseEstimate latestMeasurement = null;

        for (Vision.CameraPoseEstimate cameraMeasurement : visionMeasurements) {
            PoseEstimate visionMeasurement = cameraMeasurement.poseEstimate;
            if (!configureVisionMeasurementStdDevs(visionMeasurement)) {
                continue;
            }

            Pose2d fieldOrientedVisionPose = visionMeasurement.pose;
            poseEstimator.addVisionMeasurement(
                fieldOrientedVisionPose,
                visionMeasurement.timestampSeconds
            );

            String cameraName = cameraMeasurement.limelight.getName();
            lastVisionTimestampsByCamera.put(cameraName, visionMeasurement.timestampSeconds);
            lastAcceptedVisionPosesByCamera.put(cameraName, fieldOrientedVisionPose);
            lastVisionTimestampSeconds = Math.max(lastVisionTimestampSeconds, visionMeasurement.timestampSeconds);
            lastAcceptedVisionPose = fieldOrientedVisionPose;
            latestMeasurement = cameraMeasurement;
        }

        if (latestMeasurement == null) {
            return new Pose2d();
        }

        return lastAcceptedVisionPose;
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
        updateLimelightImuMode();
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getGyroYaw(), getModulePositions());
        addVisionMeasurementIfAvailable();

        Pose2d odometryPose = getOdometryPose();
        Pose2d estimatedPose = getPose();
        field.setRobotPose(estimatedPose);

        // SmartDashboard.putBoolean("X Enabled", Hello);

        // SmartDashboard.putBoolean("Auto Enabled", DriverStation.isAutonomousEnabled());
        // SmartDashboard.putBoolean("Auto Movement Enabled", autonMovingEnabled);
        // SmartDashboard.putBoolean("Vision Using Red Tag Filter", Constants.TeamDependentFactors.isRedTeam);
        // SmartDashboard.putNumber("Limelight IMU Mode", currentLimelightImuMode);

        // SmartDashboard.putNumber("Odometry X", odometryPose.getX());
        // SmartDashboard.putNumber("Odometry Y", odometryPose.getY());
        // SmartDashboard.putNumber("Odometry Heading", odometryPose.getRotation().getDegrees());


         // Vision Debug
        SmartDashboard.putNumber("Estimated Pose X", estimatedPose.getX());
        SmartDashboard.putNumber("Estimated Pose Y", estimatedPose.getY());
        SmartDashboard.putNumber("Estimated Pose Heading", estimatedPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Driver Forward Heading", driverForwardHeading.getDegrees());
        // SmartDashboard.putNumber("Raw MegaTag2 X", lastRawVisionPose.getX());
        // SmartDashboard.putNumber("Raw MegaTag2 Y", lastRawVisionPose.getY());
        // SmartDashboard.putNumber("Raw MegaTag2 Heading", lastRawVisionPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Raw MegaTag2 Timestamp", lastRawVisionTimestampSeconds);

        SmartDashboard.putNumber(
            "Estimator/Odometry Translation Error",
            estimatedPose.getTranslation().getDistance(odometryPose.getTranslation())
        );
        SmartDashboard.putNumber(
            "Estimator/Odometry Heading Error",
            estimatedPose.getRotation().minus(odometryPose.getRotation()).getDegrees()
        );

        // SmartDashboard.putNumber("Last Vision Pose X", lastAcceptedVisionPose.getX());
        // SmartDashboard.putNumber("Last Vision Pose Y", lastAcceptedVisionPose.getY());
        // SmartDashboard.putNumber("Last Vision Pose Heading", lastAcceptedVisionPose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Last Vision Timestamp", lastVisionTimestampSeconds);
        // SmartDashboard.putNumber("Pigeon Yaw", gyro.getYaw().getValueAsDouble());

        for (Limelight limelight : vision.getLimelights()) {
            String cameraName = limelight.getName();
            Pose2d rawPose = lastRawVisionPosesByCamera.getOrDefault(cameraName, new Pose2d());
            Pose2d acceptedPose = lastAcceptedVisionPosesByCamera.getOrDefault(cameraName, new Pose2d());
            double rawTimestamp = lastRawVisionTimestampsByCamera.getOrDefault(cameraName, -1.0);
            double acceptedTimestamp = lastVisionTimestampsByCamera.getOrDefault(cameraName, -1.0);

            // SmartDashboard.putNumber("Raw MegaTag2 " + cameraName + " X", rawPose.getX());
            // SmartDashboard.putNumber("Raw MegaTag2 " + cameraName + " Y", rawPose.getY());
            // SmartDashboard.putNumber(
            //     "Raw MegaTag2 " + cameraName + " Heading",
            //     rawPose.getRotation().getDegrees()
            // );
            // SmartDashboard.putNumber("Raw MegaTag2 " + cameraName + " Timestamp", rawTimestamp);
            // SmartDashboard.putNumber("Last Vision " + cameraName + " X", acceptedPose.getX());
            // SmartDashboard.putNumber("Last Vision " + cameraName + " Y", acceptedPose.getY());
            // SmartDashboard.putNumber(
            //     "Last Vision " + cameraName + " Heading",
            //     acceptedPose.getRotation().getDegrees()
            // );
            // SmartDashboard.putNumber("Last Vision " + cameraName + " Timestamp", acceptedTimestamp);
        }

        // Older generic pose debug kept here in case we want to re-enable it later.
        SmartDashboard.putNumber("Odometry X", getOdometryPose().getX());
        SmartDashboard.putNumber("Odometry Y", getOdometryPose().getY());
        SmartDashboard.putNumber("Estimated Pose X", getPose().getX());
        SmartDashboard.putNumber("Estimated Pose Y", getPose().getY());
        SmartDashboard.putNumber("Estimated Pose Heading", getPose().getRotation().getDegrees());

        // Per-module dashboard
        for(SwerveModule mod : mSwerveMods){
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        // SmartDashboard.putNumber("Pigeon ang vel", gyro.getAngularVelocityXDevice().getValueAsDouble());

        drivePose.set(poseEstimator.getEstimatedPosition());
        //visionLog.set(acceptedPose);
        //visionLog.set(lastAcceptedVisionPosesByCamera.getOrDefault("limelight", new Pose2d()));
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
