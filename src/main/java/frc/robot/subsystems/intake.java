package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class intake extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX intakeOutMotor;
    private final PositionDutyCycle extensionRequest = new PositionDutyCycle(0);

    private double intakePos;

    public intake() {

        intakeMotor = new TalonFX(Constants.Intake.IntakeID);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeOutMotor = new TalonFX(Constants.Intake.IntakeOutID);
        intakeOutMotor.setNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.Slot0.kP = Constants.Intake.extendP;
        extensionConfig.Slot0.kI = Constants.Intake.extendI;
        extensionConfig.Slot0.kD = Constants.Intake.extendD;
        extensionConfig.MotorOutput.PeakForwardDutyCycle = Constants.Intake.ExtendSpeed;
        extensionConfig.MotorOutput.PeakReverseDutyCycle = -Constants.Intake.ExtendSpeed;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.maxExtend;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.minExtend;
        intakeOutMotor.getConfigurator().apply(extensionConfig);
        intakeOutMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeOutMotor.setPosition(0.0);

        intakePos = Constants.Intake.minExtend;
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getExtensionPos() {
        return intakeOutMotor.getPosition().getValueAsDouble();
    }

    public void setIntakePosition(double position) {
        intakePos = position;
        clampIntakeSetPos();
    }

    public void nextArmPID() {
        clampIntakeSetPos();
        intakeOutMotor.setControl(extensionRequest.withPosition(intakePos));
    }

    private void clampIntakeSetPos() {
        intakePos = Math.max(
                Constants.Intake.minExtend,
                Math.min(Constants.Intake.maxExtend, intakePos)
        );
    }

    public void rotateArmMotor(double speed) {
        intakeOutMotor.set(speed * Constants.Intake.ExtendSpeed);
    }

    @Override
    public void periodic() {
        nextArmPID();
        SmartDashboard.putNumber("Extension value", getExtensionPos());
        SmartDashboard.putNumber("Extension target", intakePos);
    }
}
