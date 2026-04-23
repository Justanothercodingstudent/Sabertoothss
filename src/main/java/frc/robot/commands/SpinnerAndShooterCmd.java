package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.Angler;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.Vision;

public class SpinnerAndShooterCmd extends Command{
    
    private final SpinnerAndShooter shoot;
    private final intake Intake;
    private final Angler angler;
    private final XboxController Operator;
    private final XboxController Driver;
    private final Vision vision;

    private double ShootSpeed;
    private double ShootReq;
    private double SpinSpeed;
    private double Rollerspeed;
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

    private double AnglerShoot(double AnglerPos) {
        double[][] table = Constants.Spin.ShootSpeedTable;
        if (table.length == 0) {
            return ShootSpeed;
        }

        if (AnglerPos <= table[0][0]) {
            return table[0][1];
        }

        if (AnglerPos >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }

        for (int i = 1; i < table.length; i++) {
            double lowerDistance = table[i - 1][0];
            double upperDistance = table[i][0];
            if (AnglerPos <= upperDistance) {
                double lowerRotations = table[i - 1][1];
                double upperRotations = table[i][1];
                double range = upperDistance - lowerDistance;
                if (range <= 0.0) {
                    return upperRotations;
                }

                double fraction = (AnglerPos - lowerDistance) / range;
                return lowerRotations + (fraction * (upperRotations - lowerRotations));
            }
        }

        return table[table.length - 1][1];
    }

    private double ShootReq(double Speed) {
        double[][] table = Constants.Spin.ShootReqTable;
        if (table.length == 0) {
            return ShootReq;
        }

        if (Speed <= table[0][0]) {
            return table[0][1];
        }

        if (Speed >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }

        for (int i = 1; i < table.length; i++) {
            double lowerDistance = table[i - 1][0];
            double upperDistance = table[i][0];
            if (Speed <= upperDistance) {
                double lowerRotations = table[i - 1][1];
                double upperRotations = table[i][1];
                double range = upperDistance - lowerDistance;
                if (range <= 0.0) {
                    return upperRotations;
                }

                double fraction = (Speed - lowerDistance) / range;
                return lowerRotations + (fraction * (upperRotations - lowerRotations));
            }
        }

        return table[table.length - 1][1];
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean rtPressed = Operator.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean lbPressed = Operator.getLeftBumperButtonPressed();
            boolean rbPressed = Operator.getRightBumperButton();
            boolean apressed = Operator.getAButtonPressed();
            boolean xpressed = Operator.getXButton();
            boolean joyLeftPressed = Operator.getLeftStickButtonPressed();

            boolean DriverRT = Driver.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean DriverRB = Driver.getRightBumperButton();


            // SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            

            

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
                Vision.TrackedTag trackedTag = vision == null
                    ? null
                    : vision.getBestTarget(
                        Constants.TeamDependentFactors.getHubTagIds(),
                        Constants.LimelightConstants.frontCamera.name
                    );
                double trackedDistanceMeters = trackedTag == null ? -1.0 : trackedTag.distanceMeters;
                double tableShootSpeed = AnglerShoot(trackedDistanceMeters);
                double shootRequirement = ShootReq(trackedDistanceMeters);

                SmartDashboard.putNumber("Shooter Hub Tag Distance", trackedDistanceMeters);
                SmartDashboard.putString(
                    "Shooter Hub Tag Camera",
                    trackedTag == null ? "None" : trackedTag.limelight.getName()
                );

                
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;

                if (Constants.Spin.ClosedLoopShooter) {
                    shoot.setShooterRPS(tableShootSpeed, tableShootSpeed);
                } else {
                    shoot.OpenShootSpeed(ShootSpeed);
                }

                // shoot.setShooterRPS(ShootSpeed, ShootSpeed);

                if(shoot.getLeftRPS() >= shootRequirement){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (DriverRT && xpressed){
                shoot.setShooterRPS(Constants.Spin.PassingShootSpeed, Constants.Spin.PassingShootSpeed);
                if (shoot.getLeftRPS() >= Constants.Spin.PassingShootReq){
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
