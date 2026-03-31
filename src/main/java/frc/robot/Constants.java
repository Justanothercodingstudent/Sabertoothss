// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

        public static boolean isRedTeam;

        // Previous manual override implementation kept for reference only.
        // public enum TeamColorSelection {
        //     DRIVER_STATION,
        //     BLUE,
        //     RED
        // }
        
        // private static final SendableChooser<TeamColorSelection> teamColorChooser = buildTeamColorChooser();
        // public static boolean redTeam = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
        // private static SendableChooser<TeamColorSelection> buildTeamColorChooser() {
        //     SendableChooser<TeamColorSelection> chooser = new SendableChooser<>();
        //     chooser.setDefaultOption("Driver Station", TeamColorSelection.DRIVER_STATION);
        //     chooser.addOption("Blue", TeamColorSelection.BLUE);
        //     chooser.addOption("Red", TeamColorSelection.RED);
        //     return chooser;
        // }
        
        // public static SendableChooser<TeamColorSelection> getTeamColorChooser() {
        //     return teamColorChooser;
        // }
        
        // public static TeamColorSelection getSelectedTeamColor() {
        //     TeamColorSelection selectedTeamColor = teamColorChooser.getSelected();
        //     return selectedTeamColor != null ? selectedTeamColor : TeamColorSelection.DRIVER_STATION;
        // }
        
        // public static boolean isRedTeam() {
        //     TeamColorSelection selectedTeamColor = getSelectedTeamColor();
        //     if (selectedTeamColor == TeamColorSelection.RED) {
        //         redTeam = true;
        //         return true;
        //     }
        
        //     if (selectedTeamColor == TeamColorSelection.BLUE) {
        //         redTeam = false;
        //         return false;
        //     }
        
        //     redTeam = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        //     return redTeam;
        // }

        // Hub / aiming tags. These stay separate from localization filters.
        public static final double[] redHubTagIds = {
            1,
            2,
            3,  
            4,  
            5,  
            8, 
            9, 
            10,
            11   
        };

        public static final double[] blueHubTagIds = {
            18,
            19,
            20,
            21,
            24,
            25,
            26,
            27
        };

        // Full-field localization tags used by vision pose estimation on both alliances.
        public static final int[] localizationTagIds = {
            1, 2, 3, 4, 5, 6,
            7, 8, 9, 10, 11, 12,
            13, 14, 15, 16,
            17, 18, 19, 20, 21, 22,
            23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34
        };

        // public static boolean isRedTeam() {
        //     return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        // }
        

        public static double[] getHubTagIds() {
            return isRedTeam ? redHubTagIds : blueHubTagIds;
        }

        public static int[] getHubTagIdsAsInt() {
            double[] hubTagIds = getHubTagIds();
            int[] hubTagIdsAsInt = new int[hubTagIds.length];

            for (int i = 0; i < hubTagIds.length; i++) {
                hubTagIdsAsInt[i] = (int) hubTagIds[i];
            }

            return hubTagIdsAsInt;
        }

        public static int[] getLocalizationTagIds() {
            return localizationTagIds.clone();
        }
    }

    public static class LimelightConstants {
        public static final String limelightName = "limelight-front";

        public static final double XOffset = 0.3175;
        public static final double YOffset = 0.0635;
        public static final double ZOffset = 0.3048; 
        //tilt 24 degrees

        //Offset if limelight from set robot heading
        public static final double limelightHeadingOffset = 0;//180;

        public static final double minVisionTagArea = 0.05;
        public static final double maxSingleTagAmbiguity = 0.70;
        public static final double maxSingleTagDistanceMeters = 4.0;
        public static final double maxMultiTagDistanceMeters = 7.0;
        public static final double maxSingleTagPoseDeltaMeters = 1.5;
        public static final double maxMultiTagPoseDeltaMeters = 3.0;
        public static final double visionStdDevBase = 0.10;
        public static final double visionStdDevPerMeter = 0.12;
        public static final double singleTagStdDevMultiplier = 1.5;
        public static final double lowAreaStdDevMultiplier = 1.25;
        public static final double visionRotationStdDev = 9999999.0;
        public static final int limelightImuSeedMode = 3;
        public static final int limelightImuEnabledMode = 3;
        public static final double limelightImuAssistAlpha = 0.001;
    }

    public static final class Intake {
        public static final int IntakeID = 55;
        public static final double IntakeSpeed = -0.55;
        public static final double ExtendSpeed = 0.25;
        public static final double JigSpeed = 0.35;

        public static final int IntakeOutID = 13;

        public static final double maxExtend = 13.8;
        public static final double JigExtend = 6.5;
        public static final double minExtend = 0.0;

        public static final double extendP = 1;
        public static final double extendI = 0;
        public static final double extendD = 0;
    }

    // public static final class climber{
    //     //public static final int climberID = ;
    //     public static final double ClimbSpeed = 0.5;

    //     public static final double minClimb = 0;
    //     public static final double maxClimb = 90; // was 96

    //     public static  final double climbP = 1;
    //     public static final double climbI = 0;
    //     public static final double climbD = 0;
    // }

    public static final class Angler{
        public static final int AnglerID = 18;
        public static final double AngleSpeed = 0.15;

        public static double TestAngle = 0.0;

        public static final double MaxAngle = 5.5;
        public static final double Passing = 4.5;
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
            {1.0, 0.5},
            {2.16, 2.5},
            {3.14, 3.0},
            {4.42, 3.0},
        };
    }

    public static final class Spin{
        public static final boolean RightInverted = true;
        public static final boolean LeftInverted = false; 

        // IF TRUE USE NEW SHOOTER FUNCTIONALITY ELSE
        // IT USES OLD SHOOTER FUNCTIONALITY (SPEED BASED NOT RPS BASED)
        public static final boolean ClosedLoopShooter = true;

        public static final int UptakeID = 17;

        public static final int LeftFrontID = 16;
        public static final int LeftBackID = 44;

        public static final int RightFrontID = 14;
        public static final int RightBackID  = 42;

        public static final int BackRollID = 60;
        public static final int FrontRollID = 56;

        public static final double Rollerspeed = 0.25; 

        public static double PassingShootSpeed = 60;
        public static double PassingShootReq = 58;

        public static double ShootSpeed = 25;
        public static double ShootReq = 23;
        public static final double CoastSpeed = 0.05;
        public static final double SpinSpeed = -0.8;

        // We are changing our code to be RPM based as opposed to just speed based
        public static int TestTargetRPS = 0; // Based on hood angle 1.0 
        public static double ShooterToleranceRPS = 2; // 10 percent tolerance

        //PID for new shooter control method\

        public static final double LeftSideShooterkV = 0.12;
        public static final double LeftSideShooterkP = 0.62; // was .6  Causes Osscilation error if sent to 0 from  PID Needs ~.3 to run to 0
        public static final double LeftSideShooterkI = 0;
        public static final double LeftSideShooterkD = 0;

        public static final double RightSideShooterkV = 0.12;
        public static final double RightSideShooterkP = 0.62; //Causes Osscilation error if sent to 0 from PID Needs ~.3 to run to 0
        public static final double RightSideShooterkI = 0;
        public static final double RightSideShooterkD = 0;

        public static final double[][] ShootSpeedTable = {
            //{AnglePos(Rotations), Shoot Speed}
            {0.0, 45},
            {1.0, 40},
            {2.16, 45},
            {3.14, 52},
            {4.42, 59},
        };



        //TODO something with this table
        
        public static final double[][] ShootReqTable = {                         
            //{AnglePos(Rotations), Roll Speed Requirement}
            {0.0, 43},
            {1.0, 38},
            {2.16, 43},
            {3.14, 50},
            {4.42, 57},
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
      public static final double maxSpeed = 4.5; //done
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

  public static final class AutoConstants {
      public static final String defaultAutoName = "Backwards";

      public static final double translationKP = 4.5;
      public static final double translationKI = 0.00;
      public static final double translationKD = 0.0;

      public static final double rotationKP = 1.5; // Originally 0
      public static final double rotationKI = 0;
      public static final double rotationKD = 0;
  }

  public static final RobotConfig PATHPLANNER_ROBOT_CONFIG = loadPathPlannerRobotConfig();

  private static RobotConfig loadPathPlannerRobotConfig() {
      try {
          return RobotConfig.fromGUISettings();
      } catch (Exception e) {
          DriverStation.reportError(
              "Failed to load PathPlanner robot config from deploy/p%athplanner/settings.json",
              e.getStackTrace()
          );
          return null;
      }
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
