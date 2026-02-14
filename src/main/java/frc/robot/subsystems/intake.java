package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class intake extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX intakeOutMotor;

    private final PIDController extendPID;

    private double intakePos;

    public intake() {

        intakeMotor = new TalonFX(Constants.Intake.IntakeID);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeOutMotor = new TalonFX(Constants.Intake.IntakeOutID);
        intakeOutMotor.setNeutralMode(NeutralModeValue.Brake);

        extendPID = new PIDController(
                Constants.Intake.extendP,
                Constants.Intake.extendI,
                Constants.Intake.extendD
        );

        intakePos = Constants.Intake.minExtend;
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getExtensionPos() {
        double normalizedPosition = intakeOutMotor.getPosition().getValueAsDouble();

        if (normalizedPosition < 0) {
            normalizedPosition += 1.0;
        }

        return normalizedPosition;
    }

    public void setIntakePosition(double position) {
        intakePos = position;
        clampIntakeSetPos();
    }

    public void nextArmPID() {
        clampIntakeSetPos();
        setExtensionPID(intakePos);
    }

    private void clampIntakeSetPos() {
        intakePos = Math.max(
                Constants.Intake.minExtend,
                Math.min(Constants.Intake.maxExtend, intakePos)
        );
    }

    private void setExtensionPID(double position) {

        double output = extendPID.calculate(getExtensionPos(), position);

        double speedLimit = Constants.Intake.ExtendSpeed;
        output = Math.max(-speedLimit, Math.min(speedLimit, output));

        intakeOutMotor.set(-output);
    }

    public void rotateArmMotor(double speed) {
        intakeOutMotor.set(speed * Constants.Intake.ExtendSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Extension value", getExtensionPos());
    }
}
