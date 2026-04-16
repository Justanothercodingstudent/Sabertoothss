package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase {
  private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;

  private final TalonFX intakeLeftMotor;
  private final TalonFX intakeRightMotor;

  private final TalonFX intakeOutMotor;
  private final PositionDutyCycle extensionRequest = new PositionDutyCycle(0);
  private final MotorOutputConfigs extensionOutputConfig = new MotorOutputConfigs();

  private double intakePos;
  private double extendSpeed;
  private double lastDashboardUpdateSeconds = -1.0;
  private double lastExtensionMeasurement = Constants.Intake.minExtend;

  public intake() {

    extendSpeed = Math.abs(Constants.Intake.ExtendSpeed);

    intakeLeftMotor = new TalonFX(Constants.Intake.IntakeLeftID);
    intakeLeftMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeRightMotor = new TalonFX(Constants.Intake.IntakeRightID);
    intakeRightMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeOutMotor = new TalonFX(Constants.Intake.IntakeOutID, Constants.CTRE.CANIVORE_NAME);
    intakeOutMotor.setNeutralMode(NeutralModeValue.Brake);
    TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
    extensionConfig.Slot0.kP = Constants.Intake.extendP;
    extensionConfig.Slot0.kI = Constants.Intake.extendI;
    extensionConfig.Slot0.kD = Constants.Intake.extendD;
    extensionConfig.MotorOutput.PeakForwardDutyCycle = extendSpeed;
    extensionConfig.MotorOutput.PeakReverseDutyCycle = -extendSpeed;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.maxExtend;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    extensionConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.minExtend;
    intakeOutMotor.getConfigurator().apply(extensionConfig);
    intakeOutMotor.setNeutralMode(NeutralModeValue.Brake);
    // intakeOutMotor.setPosition(0.0);

    intakePos = Constants.Intake.minExtend;
  }

  public double getIntakeSpeed() {
    return intakeLeftMotor.getVelocity().getValueAsDouble();
  }

  public void setIntakeSpeed(double speed) {
    intakeLeftMotor.set(speed);
    intakeRightMotor.set(-speed);
  }

  public void IntakeWait() {}

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

  public void setExtendSpeed(double speed) {
    double requestedSpeed = Math.abs(speed);
    if (Math.abs(requestedSpeed - extendSpeed) < 1e-6) {
      return;
    }

    extendSpeed = requestedSpeed;
    intakeOutMotor.getConfigurator().refresh(extensionOutputConfig);
    extensionOutputConfig.PeakForwardDutyCycle = extendSpeed;
    extensionOutputConfig.PeakReverseDutyCycle = -extendSpeed;
    intakeOutMotor.getConfigurator().apply(extensionOutputConfig);
  }

  private void clampIntakeSetPos() {
    intakePos =
        Math.max(Constants.Intake.minExtend, Math.min(Constants.Intake.maxExtend, intakePos));
  }

  public void rotateArmMotor(double speed) {
    intakeOutMotor.set(speed * Constants.Intake.ExtendSpeed);
  }

  @Override
  public void periodic() {
    boolean disabled = DriverStation.isDisabled();
    if (!disabled) {
      nextArmPID();
    }

    double nowSeconds = Timer.getFPGATimestamp();
    if (lastDashboardUpdateSeconds >= 0.0
        && nowSeconds - lastDashboardUpdateSeconds < DASHBOARD_UPDATE_INTERVAL_SECONDS) {
      return;
    }
    lastDashboardUpdateSeconds = nowSeconds;

    if (!disabled) {
      lastExtensionMeasurement = getExtensionPos();
    }

    SmartDashboard.putNumber("Extension value", lastExtensionMeasurement);
    SmartDashboard.putNumber("Extension target", intakePos);
  }
}
