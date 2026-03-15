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
    private final Limelight limelight;

    private double AnglerPos;
    private double lastTagSeenTimestampSeconds;

    public Angler(Limelight limelight) {
        this.limelight = limelight;
        
        Angler = new TalonFX(Constants.Angler.AnglerID);
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

    /*private double distanceToMotorRotations(double distanceMeters) {
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
    }*/

    /*public void updateFromTrackedAprilTag() {
        if (limelight == null) {
            return;
        }

        int trackedTagId = Constants.Angler.trackedAprilTagId;
        double distanceMeters = limelight.getDistanceToTag(trackedTagId);
        boolean hasTrackedTag = distanceMeters >= 0.0;
        double nowSeconds = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("AprilTag " + trackedTagId + " Distance", distanceMeters);
        SmartDashboard.putBoolean("AprilTag " + trackedTagId + " Seen", hasTrackedTag);

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
    }*/

    @Override
    public void periodic(){
        //updateFromTrackedAprilTag();
        nextArmPID();
        SmartDashboard.putNumber("Angle value", getAnglePos());
        SmartDashboard.putNumber("Angle target", AnglerPos);

        SmartDashboard.putNumber("AprilTag 9 Distance", limelight.getDistanceToTag(9));
    }

}
