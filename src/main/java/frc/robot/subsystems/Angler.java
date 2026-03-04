package frc.robot.subsystems;

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

    private double AnglerPos;

    public Angler() {
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

    @Override
    public void periodic(){
        nextArmPID();
        SmartDashboard.putNumber("Angle value", getAnglePos());
        SmartDashboard.putNumber("Angle target", AnglerPos);
    }

}
