package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    public String name;
    private int[] configuredValidTagIds = new int[0];

    /*private double yUpperBound = 0;
    private double yDownBound = 480;
    private double xLeftBound = 0;
    private double xRightBound = 640;*/

    public Limelight(String name) {
        this.name = name;
    }
    public Limelight() {
        this(Constants.LimelightConstants.limelightName);
    }

    public double[] percentPosition(double[] tagInfo) { // value from 0 (top/left) to 1 (bottom/right)
        if (tagInfo == null) {
            return null;
        }
        return new double[] {
            (tagInfo[1] + 27.0) / 54.0,
            (tagInfo[2] + 20.5) / 41.0
        };
    }

    public Pose2d getAdjustedRobotPose() {
        double[] botPose = LimelightHelpers.getBotPose(Constants.LimelightConstants.limelightName);
        if (botPose.length < 6) {
            return new Pose2d(); // Return default if data is invalid
        }

        // Extract X, Y, and Rotation (Yaw) in **robot's coordinate space**
        double x = botPose[0];  // X Position (meters)
        double y = botPose[1];  // Y Position (meters)
        double yaw = botPose[5]; // Rotation in **degrees**

        // Convert yaw to Rotation2d
        Rotation2d heading = Rotation2d.fromDegrees(yaw);

        // Apply offsets from Limelight's position relative to robot
        Translation2d offset = new Translation2d(
            Constants.LimelightConstants.XOffset, 
            Constants.LimelightConstants.YOffset
        );

        // Adjust the robot’s position based on Limelight offsets
        Translation2d adjustedTranslation = new Translation2d(x, y).plus(offset);

        // Since your Limelight faces 180° backward, **rotate the heading by 180°**
        Rotation2d adjustedHeading = heading.rotateBy(Rotation2d.fromDegrees(Constants.LimelightConstants.limelightHeadingOffset));

        return new Pose2d(adjustedTranslation, adjustedHeading);
    }


    public void updateValues() {
        int[] validTagIds = Constants.TeamDependentFactors.getLocalizationTagIds();
        if (!Arrays.equals(configuredValidTagIds, validTagIds)) {
            LimelightHelpers.SetFiducialIDFiltersOverride(name, validTagIds);
            configuredValidTagIds = Arrays.copyOf(validTagIds, validTagIds.length);
        }

        LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(name);
        SmartDashboard.putBoolean("Limelight Has Target", LimelightHelpers.getTV(name));
        SmartDashboard.putNumber("Limelight Primary Tag ID", LimelightHelpers.getFiducialID(name));
        SmartDashboard.putNumber("Limelight Raw Fiducial Count", rawFiducials.length);

        // Kept for bring-up or camera troubleshooting.
        // SmartDashboard.putString("Limelight Table Name", name);
        // SmartDashboard.putBoolean("Limelight Using Red Tags", Constants.TeamDependentFactors.isRedTeam());
        // SmartDashboard.putNumber("Limelight Heartbeat", LimelightHelpers.getHeartbeat(name));
        // SmartDashboard.putNumberArray("Limelight Raw Tag IDs", getRawFiducialIds(rawFiducials));
        // SmartDashboard.putNumberArray("Limelight Localization Tag IDs", Constants.TeamDependentFactors.getLocalizationTagIds());
    }


    public void portForward() { // we dont need this but dont delete it in case we actually do need this
        for (int port = 5800; port <= 5807; port++) {
            PortForwarder.add(port, "limelight.local", port);
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

    private double[] getRawFiducialIds(LimelightHelpers.RawFiducial[] fiducials) {
        double[] tagIds = new double[fiducials.length];
        for (int i = 0; i < fiducials.length; i++) {
            tagIds[i] = fiducials[i].id;
        }

        return tagIds;
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
        double closestTag = -1;
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


    /*public PoseEstimate estimatePose() {

        LimelightHelpers.SetRobotOrientation(name, swerve.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        PoseEstimate poseEstimateNew = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        int apriltagcount = poseEstimate.tagCount;
        SmartDashboard.putNumber("apriltag count", apriltagcount);

        if (apriltagcount > 0 && fieldBoundary.isPoseWithinArea(poseEstimate.pose)) {
            if(poseEstimate.avgTagDist < 3.6576) {
                confidence = 0.5;
            } else {
                // If more than 12 ft away use MegaTag 2 use MT if less than 12
                poseEstimate = poseEstimateNew;
                confidence = 0.7;
            }
        }

        return poseEstimate;
    }*/

    @Override
    public void periodic() {
        updateValues();

        SmartDashboard.putNumber("Nearest Hub Tag", getClosestTag(Constants.TeamDependentFactors.getHubTagIds()));
        SmartDashboard.putNumber("Limelight BotPose X", getAdjustedRobotPose().getTranslation().getX());
        SmartDashboard.putNumber("Limelight BotPose Y", getAdjustedRobotPose().getTranslation().getY());
        SmartDashboard.putNumber("Limelight BotPose Yaw", getAdjustedRobotPose().getRotation().getDegrees());
    }
}
