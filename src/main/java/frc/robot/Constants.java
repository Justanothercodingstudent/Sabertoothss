// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int shooterController = 1;

    public static final double TRIGGER_THRESHOLD = 0.25;
  }


  public static final double stickDeadband = 0.08;

    public static class TeamDependentFactors {
        public static boolean forceRedTeamForTesting = false; // Set true only when intentionally forcing red

        public static final double[] validAprilTagIds = {
            2,
            3,  
            4,  
            5,  
            8, 
            9, 
            10,
            11   
        };

        public static final double[] reefIDsBlue = {
            18, // closeMiddleReefIDBlue
            19, // closeLeftReefIDBlue
            17, // closeRightReefIDBlue
            21, // farMiddleReefIDBlue
            20, // farLeftReefIDBlue
            22  // farRightReefIDBlue
        };

        public static final double[] reefIDsRed = {
            7,  // closeMiddleReefIDRed
            6,  // closeLeftReefIDRed
            8,  // closeRightReefIDRed
            10, // farMiddleReefIDRed
            11, // farLeftReefIDRed
            9   // farRightReefIDRed
        };

        public static boolean isRedAlliance() {
            if (forceRedTeamForTesting) {
                return true;
            }

            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        }

        public static double[] getReefIDs() {
            return isRedAlliance() ? reefIDsRed : reefIDsBlue;
        }


    }

    public static final class FieldConstants {
        private static final AprilTagFieldLayout rebuiltLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final double fieldLengthMeters = rebuiltLayout.getFieldLength();
        public static final double fieldWidthMeters = rebuiltLayout.getFieldWidth();
        public static final Translation2d blueHubCenter =
            new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));
        public static final Translation2d redHubCenter =
            new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.845));
        public static final Translation2d blueAutoAimTarget =
            averageTagTranslations(TeamDependentFactors.reefIDsBlue);
        public static final Translation2d redAutoAimTarget =
            averageTagTranslations(TeamDependentFactors.reefIDsRed);

        private static Translation2d averageTagTranslations(double[] tagIds) {
            double summedX = 0.0;
            double summedY = 0.0;
            int countedTags = 0;

            for (double tagId : tagIds) {
                var tagPose = rebuiltLayout.getTagPose((int) tagId);
                if (tagPose.isEmpty()) {
                    continue;
                }

                summedX += tagPose.get().getX();
                summedY += tagPose.get().getY();
                countedTags++;
            }

            if (countedTags == 0) {
                return new Translation2d();
            }

            return new Translation2d(summedX / countedTags, summedY / countedTags);
        }

        public static Translation2d getAllianceAutoAimTarget() {
            return getAllianceHubCenter();
        }

        public static Translation2d getAllianceHubCenter() {
            boolean isRedAlliance = TeamDependentFactors.isRedAlliance();

            if (!isRedAlliance) {
                return blueHubCenter;
            }

            return redHubCenter;
        }
    }

    public static final class PhotonVisionConstants {
        public static final String leftCameraName = "Arducam_OV9281_USB_Camera";
        public static final String rightCameraName = "Arducam_OV9281_USB_Camera (1)";

        // Old single-Limelight mount, no longer used.
        // public static final double cameraForwardOffsetMeters = 0.3175;
        // public static final double cameraLateralOffsetMeters = 0.0635;
        // public static final double cameraHeightMeters = 0.3048;
        // public static final double cameraPitchDegrees = 24.0;
        // public static final double cameraHeadingOffsetDegrees = 180.0;

        public static final double cameraForwardOffsetInches = 9.895;
        public static final double cameraLateralOffsetInches = 10.066;
        public static final double cameraHeightInches = 8.207;
        public static final double cameraPitchDegrees = 21.4;
        public static final double leftCameraYawDegrees = 45.0;
        public static final double rightCameraYawDegrees = -45.0;

        public static final double cameraForwardOffsetMeters =
            Units.inchesToMeters(cameraForwardOffsetInches);
        public static final double cameraLateralOffsetMeters =
            Units.inchesToMeters(cameraLateralOffsetInches);
        public static final double cameraHeightMeters =
            Units.inchesToMeters(cameraHeightInches);

        // Auto-aim uses field pose directly, so no legacy camera heading compensation is needed.
        public static final double cameraHeadingOffsetDegrees = 0.0;

        public static final Transform3d robotToLeftCamera = new Transform3d(
            new Translation3d(
                cameraForwardOffsetMeters,
                cameraLateralOffsetMeters,
                cameraHeightMeters
            ),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(cameraPitchDegrees),
                Units.degreesToRadians(leftCameraYawDegrees)
            )
        );

        public static final Transform3d robotToRightCamera = new Transform3d(
            new Translation3d(
                cameraForwardOffsetMeters,
                -cameraLateralOffsetMeters,
                cameraHeightMeters
            ),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(cameraPitchDegrees),
                Units.degreesToRadians(rightCameraYawDegrees)
            )
        );

        public static final double minVisionTargetArea = 0.05;
        public static final double maxSingleTagAmbiguity = 0.25;
        public static final double maxSingleTagDistanceMeters = 3.0;
        public static final double maxMultiTagDistanceMeters = 7.0;
        public static final double maxSingleTagPoseDeltaMeters = 1.0;
        public static final double maxMultiTagPoseDeltaMeters = 3.0;
        public static final int minVisionSeedTagCount = 2;
        public static final double poseSeedOriginToleranceMeters = 0.25;
        public static final double visionFieldBoundaryMarginMeters = 0.25;
        public static final double visionStdDevBase = 0.10;
        public static final double visionStdDevPerMeter = 0.12;
        public static final double singleTagStdDevMultiplier = 1.5;
        public static final double lowAreaStdDevMultiplier = 1.25;
        public static final double visionRotationStdDev = 9999999.0;
    }

    @Deprecated(forRemoval = false, since = "2026")
    public static class LimelightConstants {
        public static final String limelightName = PhotonVisionConstants.leftCameraName;

        // Old Limelight positioning, commented out per the PhotonVision swap.
        // public static final double XOffset = 0.3175;
        // public static final double YOffset = 0.0635;
        // public static final double ZOffset = 0.3048;
        // public static final double limelightHeadingOffset = 180.0;

        public static final double XOffset = PhotonVisionConstants.cameraForwardOffsetMeters;
        public static final double YOffset = PhotonVisionConstants.cameraLateralOffsetMeters;
        public static final double ZOffset = PhotonVisionConstants.cameraHeightMeters;
        public static final double limelightHeadingOffset =
            PhotonVisionConstants.cameraHeadingOffsetDegrees;

        public static final double minVisionTagArea = PhotonVisionConstants.minVisionTargetArea;
        public static final double maxSingleTagAmbiguity =
            PhotonVisionConstants.maxSingleTagAmbiguity;
        public static final double maxSingleTagDistanceMeters =
            PhotonVisionConstants.maxSingleTagDistanceMeters;
        public static final double maxMultiTagDistanceMeters =
            PhotonVisionConstants.maxMultiTagDistanceMeters;
        public static final double maxSingleTagPoseDeltaMeters =
            PhotonVisionConstants.maxSingleTagPoseDeltaMeters;
        public static final double maxMultiTagPoseDeltaMeters =
            PhotonVisionConstants.maxMultiTagPoseDeltaMeters;
        public static final double visionStdDevBase = PhotonVisionConstants.visionStdDevBase;
        public static final double visionStdDevPerMeter =
            PhotonVisionConstants.visionStdDevPerMeter;
        public static final double singleTagStdDevMultiplier =
            PhotonVisionConstants.singleTagStdDevMultiplier;
        public static final double lowAreaStdDevMultiplier =
            PhotonVisionConstants.lowAreaStdDevMultiplier;
        public static final double visionRotationStdDev =
            PhotonVisionConstants.visionRotationStdDev;
    }

    public static final class Intake {
        public static final int IntakeID = 55;
        public static final double IntakeSpeed = -0.50;
        public static final double ExtendSpeed = 0.1;
        public static final double JigSpeed = 0.35;

        public static final int IntakeOutID = 13;

        public static final double maxExtend = 12.5;
        public static final double JigExtend = 6.5;
        public static final double minExtend = 0.0;

        public static final double extendP = 1;
        public static final double extendI = 0;
        public static final double extendD = 0;
    }

    public static final class climber{
        public static final int climberID = 19;
        public static final double ClimbSpeed = 0.5;

        public static final double minClimb = 0;
        public static final double maxClimb = 90; // was 96

        public static  final double climbP = 1;
        public static final double climbI = 0;
        public static final double climbD = 0;
    }

    public static final class Angler{
        public static final int AnglerID = 18;
        public static final double AngleSpeed = 0.15;

        public static double TestAngle = 0.0;

        public static final double MaxAngle = 5.0;
        public static final double MinAngle = 0;

        public static final double AngleP = 0.5;
        public static final double AngleI = 0;
        public static final double AngleD = 0;

        public static final int trackedAprilTagId = 9;
        public static final double tagLostDelaySeconds = 0.5;
        public static final double noTagFallbackAngle = MinAngle;

        // Distance (m) to hood motor rotations lookup table.
        // Tune these values for your shooter.
        public static final double[][] distanceToRotationTable = {
            {0.0, 0.0},
            {1.5, 1.0},
            {2.8, 2.0},
            {4.0, 2.2},
            {5.0, 2.4},
            
        };
    }

    public static final class Spin{
        public static final int UptakeID = 17;

        public static final int LeftFrontID = 16;
        public static final int LeftBackID = 44;

        public static final int RightFrontID = 14;
        public static final int RightBackID  = 42;

        public static final int BackRollID = 60;
        public static final int FrontRollID = 56;

        public static final double Rollerspeed = 0.2; 

        public static double ShootSpeed = 0.4;
        public static final double CoastSpeed = 0.05;
        public static final double SpinSpeed = -0.8;

        public static final double[][] ShootSpeedTable = {
            //{AnglePos(Rotations), Shoot Speed}
            {0.0, 0.30},
            {1.0, 0.32},
            {1.7, 0.35},
            {2.5, 0.37},
            {3.7, 0.40}
            
        };

        public static final double[][] ShootReqTable = { 
            //{AnglePos(Rotations), Roll Speed Requirement}
            {1.0, -28.0},
            {1.7, -31.0},
            {2.5, -35.0},
        };
    }



    public static final class Swerve {
      public static final int pigeonID = 20;

      public static final int SwerveStartHeading = 0;

      public static final COTSTalonFXSwerveConstants chosenModule = 
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

      /* Drivetrain Constants */
      public static final double trackWidth = Units.inchesToMeters(24.25); // 20.966
      public static final double wheelBase = Units.inchesToMeters(24.25); // 22.8
      public static final double wheelCircumference = chosenModule.wheelCircumference;

      /* Swerve Kinematics 
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
       public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

      /* Module Gear Ratios */
      public static final double driveGearRatio = chosenModule.driveGearRatio;
      public static final double angleGearRatio = chosenModule.angleGearRatio;

      /* Motor Inverts */
      public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
      public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

      /* Angle Encoder Invert */
      public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

      /* Swerve Current Limiting */
      public static final int angleCurrentLimit = 25;
      public static final int angleCurrentThreshold = 40;
      public static final double angleCurrentThresholdTime = 0.1;
      public static final boolean angleEnableCurrentLimit = true;

      public static final int driveCurrentLimit = 60;
      public static final int driveCurrentThreshold = 70;
      public static final int driveStatorCurrentLimit = 80;
      public static final double driveCurrentThresholdTime = 0.1;
      public static final boolean driveEnableCurrentLimit = true;


      /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
       * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      /* Angle Motor PID Values */
      public static final double angleKP = chosenModule.angleKP;
      public static final double angleKI = chosenModule.angleKI;
      public static final double angleKD = chosenModule.angleKD;

      /* Drive Motor PID Values */
      public static final double driveKP = 0.5; //TODO: modifi later
      public static final double driveKI = 0.0;
      public static final double driveKD = 0.1;
      public static final double driveKF = 0.0;

      /* Drive Motor Characterization Values */
      public static final double driveKS = 2.1; //TODO: modifi later
      public static final double driveKV = 0.0;
      public static final double driveKA = 0.0;

      /* Swerve Profiling Values */
      /** Meters per Second */
      public static final double maxSpeed = 1.5; //done
      public static final double ElevatorAboveHalfMultiplier = 0.25;
      /** Radians per Second */
      public static final double maxAngularVelocity = 2 * 2 * Math.PI; //done

      /* Neutral Modes */
      public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

      //cancoder offsets
      public static final double offset0 = 74.5;
      public static final double offset1 = -149.6;
      public static final double offset2 = -169.5;
      public static final double offset3 = 18.3;

      /* Module Specific Constants */
      /* Front Left Module - Module 0 */
      public static final class Mod0 { //done
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 3;
          public static final int canCoderID = 2;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset0);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module - Module 1 */
      public static final class Mod1 { //done
          public static final int driveMotorID = 10;
          public static final int angleMotorID = 12;
          public static final int canCoderID = 11;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset1);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
      
      /* Back Left Module - Module 2 */
      public static final class Mod2 { //done
          public static final int driveMotorID = 6;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 5;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset2);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module - Module 3 */
      public static final class Mod3 { //done
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 9;
          public static final int canCoderID = 8;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(offset3);
          public static final SwerveModuleConstants constants = 
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
  }

  public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
      public static final double kMaxSpeedMetersPerSecond = 4.4; //was 3
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
  
      /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

     public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(new PIDConstants(0.1,0.01,0),new PIDConstants(0.2,0,0));
  }

  public static final RobotConfig CONFIG;
public static RobotConfig config;

  static {
      RobotConfig configTemp;
      try {
          configTemp = RobotConfig.fromGUISettings();
      } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
          configTemp = null; // or provide a default configuration umm
      }
      CONFIG = configTemp;
  }

  //public static final int elevatorMotor = ,

//old code
//* { // Load the RobotConfig from the GUI settings. You should probably
          // store this in your Constants file
         // RobotConfig config;
          //try{
         //   config = RobotConfig.fromGUISettings();
         // } catch (Exception e) {
        //    // Handle exception as needed
       //     e.printStackTrace();
       //  }
  //  }/
}
