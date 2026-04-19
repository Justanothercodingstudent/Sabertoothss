package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShootOnMoveCalculator {
  private static final double EPSILON = 1e-9;

  public static final class ShotSolution {
    private final Translation2d targetPosition;
    private final Translation2d fieldVelocityMetersPerSecond;
    private final Translation2d robotToTargetAtRelease;
    private final Translation2d shooterRelativeVelocityMetersPerSecond;
    private final Rotation2d stationaryHeading;
    private final Rotation2d compensatedHeading;
    private final double directDistanceMeters;
    private final double effectiveDistanceMeters;
    private final double flightTimeSeconds;
    private final double releaseDelaySeconds;
    private final double hoodAngleRotations;
    private final double shooterTargetRPS;
    private final double shooterFeedRPS;
    private final double shooterRpsCorrection;
    private final double estimatedLaunchAngleDegrees;
    private final double estimatedNoteExitVelocityMetersPerSecond;
    private final boolean compensationActive;

    private ShotSolution(
        Translation2d targetPosition,
        Translation2d fieldVelocityMetersPerSecond,
        Translation2d robotToTargetAtRelease,
        Translation2d shooterRelativeVelocityMetersPerSecond,
        Rotation2d stationaryHeading,
        Rotation2d compensatedHeading,
        double directDistanceMeters,
        double effectiveDistanceMeters,
        double flightTimeSeconds,
        double releaseDelaySeconds,
        double hoodAngleRotations,
        double shooterTargetRPS,
        double shooterFeedRPS,
        double shooterRpsCorrection,
        double estimatedLaunchAngleDegrees,
        double estimatedNoteExitVelocityMetersPerSecond,
        boolean compensationActive) {
      this.targetPosition = targetPosition;
      this.fieldVelocityMetersPerSecond = fieldVelocityMetersPerSecond;
      this.robotToTargetAtRelease = robotToTargetAtRelease;
      this.shooterRelativeVelocityMetersPerSecond = shooterRelativeVelocityMetersPerSecond;
      this.stationaryHeading = stationaryHeading;
      this.compensatedHeading = compensatedHeading;
      this.directDistanceMeters = directDistanceMeters;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
      this.flightTimeSeconds = flightTimeSeconds;
      this.releaseDelaySeconds = releaseDelaySeconds;
      this.hoodAngleRotations = hoodAngleRotations;
      this.shooterTargetRPS = shooterTargetRPS;
      this.shooterFeedRPS = shooterFeedRPS;
      this.shooterRpsCorrection = shooterRpsCorrection;
      this.estimatedLaunchAngleDegrees = estimatedLaunchAngleDegrees;
      this.estimatedNoteExitVelocityMetersPerSecond = estimatedNoteExitVelocityMetersPerSecond;
      this.compensationActive = compensationActive;
    }

    public Translation2d getTargetPosition() {
      return targetPosition;
    }

    public Translation2d getFieldVelocityMetersPerSecond() {
      return fieldVelocityMetersPerSecond;
    }

    public Translation2d getRobotToTargetAtRelease() {
      return robotToTargetAtRelease;
    }

    public Translation2d getShooterRelativeVelocityMetersPerSecond() {
      return shooterRelativeVelocityMetersPerSecond;
    }

    public Rotation2d getStationaryHeading() {
      return stationaryHeading;
    }

    public Rotation2d getCompensatedHeading() {
      return compensatedHeading;
    }

    public double getDirectDistanceMeters() {
      return directDistanceMeters;
    }

    public double getEffectiveDistanceMeters() {
      return effectiveDistanceMeters;
    }

    public double getFlightTimeSeconds() {
      return flightTimeSeconds;
    }

    public double getReleaseDelaySeconds() {
      return releaseDelaySeconds;
    }

    public double getHoodAngleRotations() {
      return hoodAngleRotations;
    }

    public double getShooterTargetRPS() {
      return shooterTargetRPS;
    }

    public double getShooterFeedRPS() {
      return shooterFeedRPS;
    }

    public double getShooterRpsCorrection() {
      return shooterRpsCorrection;
    }

    public double getEstimatedLaunchAngleDegrees() {
      return estimatedLaunchAngleDegrees;
    }

    public double getEstimatedNoteExitVelocityMetersPerSecond() {
      return estimatedNoteExitVelocityMetersPerSecond;
    }

    public boolean isCompensationActive() {
      return compensationActive;
    }

    public double getLeadAngleDegrees() {
      return normalizeDegrees(compensatedHeading.minus(stationaryHeading).getDegrees());
    }

    public double getYawErrorDegrees(Rotation2d robotHeading) {
      return normalizeDegrees(robotHeading.minus(compensatedHeading).getDegrees());
    }

    public boolean isRobotAimed(Rotation2d robotHeading) {
      return Math.abs(getYawErrorDegrees(robotHeading))
          <= Constants.ShootOnMove.feedAimToleranceDegrees;
    }
  }

  private ShootOnMoveCalculator() {}

  public static ShotSolution calculate(Pose2d robotPose, ChassisSpeeds robotRelativeSpeeds) {
    Translation2d fieldVelocity = new Translation2d();
    if (robotPose != null && robotRelativeSpeeds != null) {
      fieldVelocity =
          new Translation2d(
                  robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
              .rotateBy(robotPose.getRotation());
    }

    return calculate(robotPose, fieldVelocity);
  }

  public static ShotSolution calculateLineOfSight(
      double yawErrorDegrees, double distanceMeters, ChassisSpeeds robotRelativeSpeeds) {
    Translation2d robotRelativeVelocity = new Translation2d();
    if (robotRelativeSpeeds != null) {
      robotRelativeVelocity =
          new Translation2d(
              robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    }

    return calculateLineOfSight(yawErrorDegrees, distanceMeters, robotRelativeVelocity);
  }

  public static ShotSolution calculateLineOfSight(
      double yawErrorDegrees, double distanceMeters, Translation2d robotRelativeVelocity) {
    if (!Double.isFinite(yawErrorDegrees)) {
      yawErrorDegrees = 0.0;
    }
    if (!Double.isFinite(distanceMeters)) {
      distanceMeters = 0.0;
    }
    if (robotRelativeVelocity == null) {
      robotRelativeVelocity = new Translation2d();
    }

    Pose2d robotPose = new Pose2d();
    Translation2d targetPosition =
        new Translation2d(Math.max(distanceMeters, 0.0), Rotation2d.fromDegrees(-yawErrorDegrees));
    return calculate(robotPose, targetPosition, robotRelativeVelocity);
  }

  public static ShotSolution calculate(Pose2d robotPose, Translation2d fieldVelocity) {
    return calculate(robotPose, Constants.TeamDependentFactors.hubPosition(), fieldVelocity);
  }

  public static ShotSolution calculate(
      Pose2d robotPose, Translation2d targetPosition, Translation2d fieldVelocity) {
    if (robotPose == null) {
      robotPose = new Pose2d();
    }
    if (targetPosition == null) {
      targetPosition = Constants.TeamDependentFactors.hubPosition();
    }
    if (fieldVelocity == null) {
      fieldVelocity = new Translation2d();
    }

    Translation2d directRobotToTarget = targetPosition.minus(robotPose.getTranslation());
    double directDistanceMeters = directRobotToTarget.getNorm();
    Rotation2d stationaryHeading = getTranslationHeading(directRobotToTarget, robotPose);

    Translation2d compensatedFieldVelocity = getCompensatedFieldVelocity(fieldVelocity);
    boolean compensationActive = compensatedFieldVelocity.getNorm() > EPSILON;
    double releaseDelaySeconds =
        compensationActive ? Constants.ShootOnMove.releaseDelaySeconds : 0.0;

    Translation2d robotTranslationAtRelease =
        robotPose.getTranslation().plus(compensatedFieldVelocity.times(releaseDelaySeconds));
    Translation2d robotToTargetAtRelease = targetPosition.minus(robotTranslationAtRelease);

    double flightTimeSeconds =
        Constants.ShootOnMove.getFlightTimeForDistance(robotToTargetAtRelease.getNorm());
    Translation2d shooterRelativeVelocity = new Translation2d();
    double effectiveDistanceMeters = directDistanceMeters;

    for (int i = 0; i < Constants.ShootOnMove.solverIterations; i++) {
      shooterRelativeVelocity =
          divide(robotToTargetAtRelease, flightTimeSeconds).minus(compensatedFieldVelocity);
      effectiveDistanceMeters = shooterRelativeVelocity.getNorm() * flightTimeSeconds;
      flightTimeSeconds = Constants.ShootOnMove.getFlightTimeForDistance(effectiveDistanceMeters);
    }

    shooterRelativeVelocity =
        divide(robotToTargetAtRelease, flightTimeSeconds).minus(compensatedFieldVelocity);
    effectiveDistanceMeters = shooterRelativeVelocity.getNorm() * flightTimeSeconds;

    Rotation2d compensatedHeading =
        shooterRelativeVelocity.getNorm() > EPSILON
            ? shooterRelativeVelocity.getAngle()
            : stationaryHeading;

    double hoodAngleRotations =
        Constants.Angler.getAngleRotationsForDistance(effectiveDistanceMeters);
    double tableShooterTargetRPS = Constants.Spin.getShooterRPSForAngle(hoodAngleRotations);
    double tableShooterFeedRPS = Constants.Spin.getShooterFeedRPSForAngle(hoodAngleRotations);

    double stationaryFlightTimeSeconds =
        Constants.ShootOnMove.getFlightTimeForDistance(directDistanceMeters);
    double stationaryHorizontalSpeed =
        stationaryFlightTimeSeconds > EPSILON
            ? directDistanceMeters / stationaryFlightTimeSeconds
            : 0.0;
    double requiredHorizontalSpeed = shooterRelativeVelocity.getNorm();
    double shooterRpsCorrection =
        MathUtil.clamp(
            (requiredHorizontalSpeed - stationaryHorizontalSpeed)
                * Constants.ShootOnMove.shooterRpsPerMeterPerSecond,
            -Constants.ShootOnMove.maxShooterRpsCorrection,
            Constants.ShootOnMove.maxShooterRpsCorrection);

    double shooterTargetRPS =
        MathUtil.clamp(
            tableShooterTargetRPS + shooterRpsCorrection,
            Constants.ShootOnMove.minShooterTargetRPS,
            Constants.ShootOnMove.maxShooterTargetRPS);
    double shooterFeedRPS =
        MathUtil.clamp(
            tableShooterFeedRPS + shooterRpsCorrection,
            Constants.ShootOnMove.minShooterTargetRPS,
            Constants.ShootOnMove.maxShooterTargetRPS);
    double estimatedLaunchAngleDegrees =
        Constants.ShootOnMove.getEstimatedLaunchAngleDegrees(hoodAngleRotations);
    double estimatedNoteExitVelocityMetersPerSecond =
        Constants.ShootOnMove.getEstimatedNoteExitVelocityMetersPerSecond(shooterTargetRPS);

    return new ShotSolution(
        targetPosition,
        compensatedFieldVelocity,
        robotToTargetAtRelease,
        shooterRelativeVelocity,
        stationaryHeading,
        compensatedHeading,
        directDistanceMeters,
        effectiveDistanceMeters,
        flightTimeSeconds,
        releaseDelaySeconds,
        hoodAngleRotations,
        shooterTargetRPS,
        shooterFeedRPS,
        shooterRpsCorrection,
        estimatedLaunchAngleDegrees,
        estimatedNoteExitVelocityMetersPerSecond,
        compensationActive);
  }

  private static Translation2d getCompensatedFieldVelocity(Translation2d fieldVelocity) {
    if (!Constants.ShootOnMove.enabled) {
      return new Translation2d();
    }

    double speedMetersPerSecond = fieldVelocity.getNorm();
    if (speedMetersPerSecond < Constants.ShootOnMove.minCompensationSpeedMetersPerSecond) {
      return new Translation2d();
    }

    if (speedMetersPerSecond <= Constants.ShootOnMove.maxCompensationSpeedMetersPerSecond) {
      return fieldVelocity;
    }

    return fieldVelocity.times(
        Constants.ShootOnMove.maxCompensationSpeedMetersPerSecond / speedMetersPerSecond);
  }

  private static Rotation2d getTranslationHeading(Translation2d translation, Pose2d fallbackPose) {
    if (translation.getNorm() > EPSILON) {
      return translation.getAngle();
    }

    return fallbackPose.getRotation();
  }

  private static Translation2d divide(Translation2d translation, double scalar) {
    if (Math.abs(scalar) < EPSILON) {
      return new Translation2d();
    }

    return new Translation2d(translation.getX() / scalar, translation.getY() / scalar);
  }

  private static double normalizeDegrees(double degrees) {
    return MathUtil.inputModulus(degrees, -180.0, 180.0);
  }
}
