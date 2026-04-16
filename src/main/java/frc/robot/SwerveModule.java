package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CoreCANcoder angleEncoder;
  private CTREConfigs ctreConfigs = new CTREConfigs();

  private final SimpleMotorFeedforward driveFeedForward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CoreCANcoder(moduleConstants.cancoderID, Constants.CTRE.CANIVORE_NAME);
    // angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);
    angleEncoder.getConfigurator().apply(ctreConfigs.swerveCANcoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Constants.CTRE.CANIVORE_NAME);
    mAngleMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Constants.CTRE.CANIVORE_NAME);
    mDriveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);
    mDriveMotor.getConfigurator().setPosition(0.0);
    mDriveMotor.set(0);

    // mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public TalonFX[] getTalons() {
    TalonFX[] talons = {mAngleMotor, mDriveMotor};
    return talons;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getAngle());
    mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity =
          Conversions.MPSToRPS(
              desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  } // Hi this is a test

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference),
        getAngle());
  }

  public SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics();
  }
}
