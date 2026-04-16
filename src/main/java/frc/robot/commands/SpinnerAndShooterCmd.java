package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.Angler;
import frc.robot.Constants;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.Vision;

public class SpinnerAndShooterCmd extends Command{
    private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;
    
    private final SpinnerAndShooter shoot;
    private final intake Intake;
    private final Angler angler;
    private final XboxController Operator;
    private final XboxController Driver;
    private final Vision vision;

    private double SpinSpeed;
    private double Rollerspeed;
    private double lastDashboardUpdateSeconds = -1.0;
    public SpinnerAndShooterCmd(
        SpinnerAndShooter shoot,
        XboxController Operator,
        Angler angler,
        intake Intake,
        Vision vision,
        XboxController Driver
    ){
        this.shoot = shoot;
        addRequirements(this.shoot);

        this.angler = angler;
        this.Intake = Intake;
        this.vision = vision;
        this.Operator = Operator;
        this.Driver = Driver;
    }

     @Override
    public void initialize() {
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {
            double nowSeconds = Timer.getFPGATimestamp();
            boolean updateDashboard = lastDashboardUpdateSeconds < 0.0
                || nowSeconds - lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_INTERVAL_SECONDS;
            if (updateDashboard) {
                lastDashboardUpdateSeconds = nowSeconds;
            }

            boolean rtPressed = Operator.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rbPressed = Operator.getRightBumperButton();
            boolean xpressed = Operator.getXButton();

            boolean DriverRT = Driver.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;


            if (updateDashboard) {
                SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging
            }

            

            

            // if (rbPressed){
            //     Constants.Spin.ShootSpeed -= 1;
            //     Constants.Spin.ShootReq -= 1;
            // } else if (lbPressed){
            //     Constants.Spin.ShootSpeed += 1;
            //     Constants.Spin.ShootReq += 1;
            // }

            // double ShootSpeed = Constants.Spin.ShootSpeed;
            // ShootReq = Constants.Spin.ShootReq;
            // SmartDashboard.putNumber("Shoot Speed Target", ShootSpeed);

            if (rtPressed && !xpressed){
                Vision.TrackedTag trackedTag = updateDashboard && vision != null
                    ? vision.getBestTarget(
                        Constants.TeamDependentFactors.getHubTagIds(),
                        Constants.LimelightConstants.frontCamera.name
                    )
                    : null;
                double trackedDistanceMeters = trackedTag == null ? -1.0 : trackedTag.distanceMeters;
                double anglerTarget = angler.getAngleTarget();
                double tableShootSpeed = Constants.Spin.getShooterRPSForAngle(anglerTarget);
                double shootRequirement = Constants.Spin.getShooterFeedRPSForAngle(anglerTarget);

                if (updateDashboard) {
                    SmartDashboard.putNumber("Shooter Hub Tag Distance", trackedDistanceMeters);
                    SmartDashboard.putNumber("Shooter Angler Target", anglerTarget);
                    SmartDashboard.putNumber("Shoot Speed Target", tableShootSpeed);
                    SmartDashboard.putNumber("Shoot Feed Requirement", shootRequirement);
                    SmartDashboard.putString(
                        "Shooter Hub Tag Camera",
                        trackedTag == null ? "None" : trackedTag.limelight.getName()
                    );
                }

                
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;

                if (Constants.Spin.ClosedLoopShooter) {
                    shoot.setShooterRPS(tableShootSpeed, tableShootSpeed);
                } else {
                    shoot.OpenShootSpeed(Constants.Spin.ShootSpeed);
                }

                // shoot.setShooterRPS(ShootSpeed, ShootSpeed);

                if(shoot.isAtOrAboveRPS(shootRequirement)){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (DriverRT && xpressed){
                shoot.setShooterRPS(Constants.Spin.PassingShootSpeed, Constants.Spin.PassingShootSpeed);
                if (shoot.isAtOrAboveRPS(Constants.Spin.PassingShootReq)){
                    shoot.roller(Constants.Spin.Rollerspeed);
                    shoot.SpinSpeed(Constants.Spin.SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (rbPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(-Rollerspeed);
            } else {
                shoot.SpinSpeed(0);
                shoot.OpenShootSpeed(0);
                shoot.roller(0);
            } 
            
            
            
            
            
        }
        
    }

}
