package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Angler extends SubsystemBase{
    private final TalonFX Angler;
    private final PositionDutyCycle AngleRequest = new PositionDutyCycle(0);
    private final Vision vision;

    private double AnglerPos;
    private double lastTagSeenTimestampSeconds;

    public Angler(Vision vision) {
        this.vision = vision;
        
        Angler = new TalonFX(Constants.Angler.AnglerID, Constants.CTRE.CANIVORE_NAME);
        Angler.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration AngleConfig = new TalonFXConfiguration();
        AngleConfig.Slot0.kP = Constants.Angler.AngleP;
        AngleConfig.Slot0.kI = Constants.Angler.AngleI;
        AngleConfig.Slot0.kD = Constants.Angler.AngleD;
        AngleConfig.MotorOutput.PeakForwardDutyCycle = Constants.Angler.AngleSpeed;
        AngleConfig.MotorOutput.PeakReverseDutyCycle = -Constants.Angler.AngleSpeed;
        AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        AngleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Angler.MaxAngle;
        AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        AngleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Angler.MinAngle;
        Angler.getConfigurator().apply(AngleConfig);
        Angler.setNeutralMode(NeutralModeValue.Brake);
       
        Angler.setPosition(0.0);
        AnglerPos = Constants.Angler.MinAngle;
        lastTagSeenTimestampSeconds = Timer.getFPGATimestamp();
    }

    public Angler() {
        this(null);
    }

    public void AngleSpeed(double speed){
        Angler.set(speed);
    }
    
    public void setAnglePosition(double position) {
        AnglerPos = position;
        clampAngleSetPos(); 
    }

   

    public void clampAngleSetPos() {
        AnglerPos = Math.max(
             Constants.Angler.MinAngle, // Ensure minimum position
             Math.min(Constants.Angler.MaxAngle, AnglerPos) // Ensure maximum position
        );
        
    }

    public void nextArmPID() {
        clampAngleSetPos();
        Angler.setControl(AngleRequest.withPosition(AnglerPos));
    }

    public double getAnglePos(){
        return Angler.getPosition().getValueAsDouble();
    }

    public void rotateArmMotor(double speed) {
        Angler.set(speed * Constants.Angler.AngleSpeed);
    }

    private double distanceToMotorRotations(double distanceMeters) {
        double[][] table = Constants.Angler.distanceToRotationTable;
        if (table.length == 0) {
            return AnglerPos;
        }

        if (distanceMeters <= table[0][0]) {
            return table[0][1];
        }
        if (distanceMeters >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }

        for (int i = 1; i < table.length; i++) {
            double lowerDistance = table[i - 1][0];
            double upperDistance = table[i][0];
            if (distanceMeters <= upperDistance) {
                double lowerRotations = table[i - 1][1];
                double upperRotations = table[i][1];
                double range = upperDistance - lowerDistance;
                if (range <= 0.0) {
                    return upperRotations;
                }

                double fraction = (distanceMeters - lowerDistance) / range;
                return lowerRotations + (fraction * (upperRotations - lowerRotations));
            }
        }

        return table[table.length - 1][1];
    }

    public void updateFromTrackedAprilTag() {
        Vision.TrackedTag trackedTag = getTrackedHubTag();
        if (trackedTag == null) {
            return;
        }

        double trackedTagId = trackedTag.tagId;
        double distanceMeters = trackedTag.distanceMeters;
        boolean hasTrackedTag = distanceMeters >= 0.0;
        double nowSeconds = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("AprilTag " + trackedTagId + " Distance", distanceMeters);
        SmartDashboard.putBoolean("AprilTag " + trackedTagId + " Seen", hasTrackedTag);
        SmartDashboard.putString(
            "Tracked AprilTag Camera",
            trackedTag.limelight.getName()
        );

        if (hasTrackedTag) {
            lastTagSeenTimestampSeconds = nowSeconds;
            setAnglePosition(distanceToMotorRotations(distanceMeters));
            return;
         } 

        double timeSinceLastSeen = nowSeconds - lastTagSeenTimestampSeconds;
        SmartDashboard.putNumber("AprilTag " + trackedTagId + " Time Since Seen", timeSinceLastSeen);
        if (timeSinceLastSeen >= Constants.Angler.tagLostDelaySeconds) {
            setAnglePosition(Constants.Angler.noTagFallbackAngle);
        }
    }

    private Vision.TrackedTag getTrackedHubTag() {
        if (vision == null) {
            return null;
        }

        return vision.getBestTarget(
            Constants.TeamDependentFactors.getHubTagIds(),
            Constants.LimelightConstants.frontCamera.name
        );
    }

    @Override
    public void periodic(){
        nextArmPID();
        SmartDashboard.putNumber("Angle value", getAnglePos());
        SmartDashboard.putNumber("Angle target", AnglerPos);
        Vision.TrackedTag trackedTag = getTrackedHubTag();
        SmartDashboard.putNumber(
            "Nearest Hub AprilTag Distance",
            trackedTag == null ? -1.0 : trackedTag.distanceMeters
        );
        SmartDashboard.putNumber(
            "Nearest Hub AprilTag ID",
            trackedTag == null ? -1.0 : trackedTag.tagId
        );
        SmartDashboard.putString(
            "Nearest Hub AprilTag Camera",
            trackedTag == null ? "None" : trackedTag.limelight.getName()
        );
    }
//helllloooooo
}
