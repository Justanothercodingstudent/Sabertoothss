package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final Constants.LimelightConstants.CameraConfig config;
    private final String name;
    private int[] configuredValidTagIds = new int[0];

    public Limelight(Constants.LimelightConstants.CameraConfig config) {
        this.config = config;
        this.name = config.name;
    }

    public Limelight() {
        this(Constants.LimelightConstants.frontCamera);
    }

    public String getName() {
        return name;
    }

    public String getDashboardPrefix() {
        return config.dashboardPrefix;
    }

    public double[] percentPosition(double[] tagInfo) {
        if (tagInfo == null) {
            return null;
        }

        return new double[] {
            (tagInfo[1] + 27.0) / 54.0,
            (tagInfo[2] + 20.5) / 41.0
        };
    }

    public Pose2d getAdjustedRobotPose() {
        double[] botPose = LimelightHelpers.getBotPose(name);
        if (botPose.length < 6) {
            return new Pose2d();
        }

        double x = botPose[0];
        double y = botPose[1];
        double yaw = botPose[5];

        Rotation2d heading = Rotation2d.fromDegrees(yaw);
        Translation2d offset = new Translation2d(config.xOffset, config.yOffset);
        Translation2d adjustedTranslation = new Translation2d(x, y).plus(offset);
        Rotation2d adjustedHeading = heading.rotateBy(
            Rotation2d.fromDegrees(config.headingOffsetDegrees)
        );

        return new Pose2d(adjustedTranslation, adjustedHeading);
    }

    public void pushRobotOrientation(Rotation2d fieldHeading) {
        if (!Double.isFinite(fieldHeading.getRadians())) {
            return;
        }

        LimelightHelpers.SetRobotOrientation(
            name,
            fieldHeading.getDegrees(),
            0,
            0,
            0,
            0,
            0
        );
    }

    public void setIMUMode(int imuMode) {
        LimelightHelpers.SetIMUAssistAlpha(name, Constants.LimelightConstants.limelightImuAssistAlpha);
        LimelightHelpers.SetIMUMode(name, imuMode);
    }

    public PoseEstimate getMegaTag2PoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    public void updateValues() {
        applyHubCenterOffset();

        int[] validTagIds = Constants.TeamDependentFactors.getLocalizationTagIds();
        if (!Arrays.equals(configuredValidTagIds, validTagIds)) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, validTagIds);
            configuredValidTagIds = Arrays.copyOf(validTagIds, validTagIds.length);
        }

        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(name);
        SmartDashboard.putBoolean(dashboardKey("Has Target"), LimelightHelpers.getTV(name));
        SmartDashboard.putNumber(dashboardKey("Primary Tag ID"), LimelightHelpers.getFiducialID(name));
        SmartDashboard.putNumber(dashboardKey("Raw Fiducial Count"), rawFiducials.length);
    }

    public static Translation2d getHubCenterOffset(int tagId) {
        switch (tagId) {
            case 2:  return new Translation2d(-0.6033770, 0.0001016);
            case 3:  return new Translation2d(-0.6036564, 0.3555746);
            case 4:  return new Translation2d(0.6036564, 0.0000254);
            case 5:  return new Translation2d(-0.6034278, -0.0001016);
            case 8:  return new Translation2d(-0.3554984, -0.2034278);
            case 9:  return new Translation2d(-0.6036564, 0.3056254);
            case 10: return new Translation2d(-0.6036564, 0.0000254);
            case 11: return new Translation2d(-0.2554984, 0.0337700);
            case 18: return new Translation2d(-0.0001524, 0.6034278);
            case 19: return new Translation2d(-0.6037072, 0.3556254);
            case 20: return new Translation2d(-0.6037072, 0.0000254);
            case 21: return new Translation2d(0.6033770, -0.0001524);
            case 24: return new Translation2d(-0.6033770, -0.3554476);
            case 25: return new Translation2d(-0.6036056, 0.3555746);
            case 26: return new Translation2d(-0.6036056, 0.0000254);
            case 27: return new Translation2d(-0.6034278, 0.3554476);
            default:
                throw new IllegalArgumentException("Tag " + tagId + " is not a hub tag");
        }
    }

    private String dashboardKey(String key) {
        return config.dashboardPrefix + " " + key;
    }

    public void getRecording(double MatchEnd){
        if (MatchEnd <= 140){
            LimelightHelpers.triggerRewindCapture("limelight-front", 20.0);
            LimelightHelpers.triggerRewindCapture("limelight-left", 20.0);
        }
    }

    private boolean isValidTagId(int tagId, double[] validTagIds) {
        for (double validTagId : validTagIds) {
            if (tagId == (int) validTagId) {
                return true;
            }
        }

        return false;
    }

    public double getCurrentTargetTagId() {
        if (!LimelightHelpers.getTV(name)) {
            return -1.0;
        }

        int tagId = (int) LimelightHelpers.getFiducialID(name);
        return tagId > 0 ? tagId : -1.0;
    }

    public double getCurrentHubTagId() {
        int tagId = (int) getCurrentTargetTagId();
        if (tagId <= 0 || !isValidTagId(tagId, Constants.TeamDependentFactors.getHubTagIds())) {
            return -1.0;
        }

        return tagId;
    }

    public Translation2d getCurrentHubCenterOffset() {
        int tagId = (int) getCurrentHubTagId();
        if (tagId <= 0) {
            return new Translation2d();
        }

        try {
            return getHubCenterOffset(tagId);
        } catch (IllegalArgumentException ex) {
            return new Translation2d();
        }
    }

    public void applyHubCenterOffset() {
        Translation2d offset = getCurrentHubCenterOffset();
        LimelightHelpers.setFiducial3DOffset(name, offset.getX(), offset.getY(), 0.0);

        SmartDashboard.putNumber(dashboardKey("Offset Tag ID"), LimelightHelpers.getFiducialID(name));
        SmartDashboard.putNumber(dashboardKey("Fiducial Offset X"), offset.getX());
        SmartDashboard.putNumber(dashboardKey("Fiducial Offset Y"), offset.getY());
    }

    public void portForward() {
        String hostName = name + ".local";
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, hostName, port);
        }
    }

    public void stopAprilTagDetector() {}

    private LimelightHelpers.RawFiducial getRawFiducial(int tagId) {
        for (LimelightHelpers.RawFiducial fiducial : LimelightHelpers.getRawFiducials(name)) {
            if (fiducial.id == tagId) {
                return fiducial;
            }
        }

        return null;
    }

    private boolean isTrackedTag(double[] validTagIds, int tagId) {
        for (double validTagId : validTagIds) {
            if ((int) validTagId == tagId) {
                return true;
            }
        }

        return false;
    }

    public double getClosestTag(double[] validTagIds) {
        double closestTag = -1.0;
        double closestDistanceMeters = Double.MAX_VALUE;

        for (LimelightHelpers.RawFiducial fiducial : LimelightHelpers.getRawFiducials(name)) {
            for (double validId : validTagIds) {
                if (fiducial.id == (int) validId && fiducial.distToCamera < closestDistanceMeters) {
                    closestTag = fiducial.id;
                    closestDistanceMeters = fiducial.distToCamera;
                }
            }
        }

        if (closestTag >= 0.0) {
            return closestTag;
        }

        int primaryTagId = (int) LimelightHelpers.getFiducialID(name);
        if (LimelightHelpers.getTV(name) && isTrackedTag(validTagIds, primaryTagId)) {
            return primaryTagId;
        }

        return -1.0;
    }

    public double getDistanceToTag(double targetTagId) {
        LimelightHelpers.RawFiducial fiducial = getRawFiducial((int) targetTagId);
        if (fiducial != null) {
            return fiducial.distToCamera;
        }

        if (LimelightHelpers.getTV(name) && (int) targetTagId == (int) LimelightHelpers.getFiducialID(name)) {
            return LimelightHelpers.getTargetPose3d_CameraSpace(name).getTranslation().getNorm();
        }

        return -1.0;
    }

    public double[] getTarget(int id) {
        LimelightHelpers.RawFiducial fiducial = getRawFiducial(id);
        if (fiducial != null) {
            return new double[] {
                fiducial.id,
                fiducial.txnc,
                fiducial.tync
            };
        }

        if (LimelightHelpers.getTV(name) && id == (int) LimelightHelpers.getFiducialID(name)) {
            return new double[] {
                id,
                LimelightHelpers.getTXNC(name),
                LimelightHelpers.getTYNC(name)
            };
        }

        return null;
    }

    @Override
    public void periodic() {
        updateValues();

        SmartDashboard.putNumber(
            dashboardKey("Nearest Hub Tag"),
            getClosestTag(Constants.TeamDependentFactors.getHubTagIds())
        );
        SmartDashboard.putNumber(
            dashboardKey("BotPose X"),
            getAdjustedRobotPose().getTranslation().getX()
        );
        SmartDashboard.putNumber(
            dashboardKey("BotPose Y"),
            getAdjustedRobotPose().getTranslation().getY()
        );
        SmartDashboard.putNumber(
            dashboardKey("BotPose Yaw"),
            getAdjustedRobotPose().getRotation().getDegrees()
        );
    }
}
