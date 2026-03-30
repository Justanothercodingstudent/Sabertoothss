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
import frc.robot.subsystems.Limelight;

public class SpinnerAndShooterCmd extends Command{
    
    private final SpinnerAndShooter shoot;
    private final intake Intake;
    private final Angler angler;
    private final XboxController xbox;
    private final Limelight limelight;

    private double ShootSpeed;
    private double ShootReq;
    private double SpinSpeed;
    private double Rollerspeed;
    public SpinnerAndShooterCmd(SpinnerAndShooter shoot, XboxController xbox, Angler angler, intake Intake, Limelight limelight){
        this.shoot = shoot;
        addRequirements(this.shoot);

        this.angler = angler;
        this.Intake = Intake;
        this.limelight = limelight;
        this.xbox = xbox;
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

            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean lbPressed = xbox.getLeftBumperButtonPressed();
            boolean rbPressed = xbox.getRightBumperButtonPressed();
            boolean apressed = xbox.getAButtonPressed();
            boolean xpressed = xbox.getXButtonPressed();
            boolean joyLeftPressed = xbox.getLeftStickButtonPressed();

            SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            

            

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

            if (rtPressed){

                double TableShootSpeed = AnglerShoot(limelight.getDistanceToTag(limelight.getClosestTag(Constants.TeamDependentFactors.getHubTagIds())));
                double ShootReq = ShootReq(limelight.getDistanceToTag(limelight.getClosestTag(Constants.TeamDependentFactors.getHubTagIds())));

                
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;

                if (Constants.Spin.ClosedLoopShooter) {
                    shoot.setShooterRPS(TableShootSpeed, TableShootSpeed);
                } else {
                    shoot.OpenShootSpeed(ShootSpeed);
                }

                // shoot.setShooterRPS(ShootSpeed, ShootSpeed);

                if(shoot.getLeftRPS() >= ShootReq){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (rtPressed && xpressed){
                shoot.setShooterRPS(Constants.Spin.PassingShootSpeed, Constants.Spin.PassingShootSpeed);
                if (shoot.getLeftRPS() >= Constants.Spin.PassingShootReq){
                    shoot.roller(Constants.Spin.Rollerspeed);
                    shoot.SpinSpeed(Constants.Spin.SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (!rtPressed && !xpressed){
                shoot.SpinSpeed(0);
                shoot.OpenShootSpeed(0);
                shoot.roller(0);
            } 
            
            if (joyLeftPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(-Rollerspeed);
            }
            
            
            
        }
        
    }

}