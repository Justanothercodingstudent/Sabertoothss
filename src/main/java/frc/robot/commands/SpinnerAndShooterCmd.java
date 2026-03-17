package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.Angler;
import frc.robot.Constants;

public class SpinnerAndShooterCmd extends Command{
    
    private final SpinnerAndShooter shoot;
    private final Angler angler;
    private final XboxController xbox;

    private double ShootSpeed;
    private double ShootReq;
    private double SpinSpeed;
    private double Rollerspeed;
    public SpinnerAndShooterCmd(SpinnerAndShooter shoot, XboxController xbox, Angler angler){
        this.shoot = shoot;
        addRequirements(this.shoot);

        this.angler = angler;

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
            boolean ltPressed = xbox.getLeftTriggerAxis()  > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean apressed = xbox.getAButtonPressed();
            boolean xpressed = xbox.getXButtonPressed();
            boolean joyLeftPressed = xbox.getLeftStickButtonPressed();

            SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            /*if(apressed){
                Constants.Spin.ShootSpeed += 0.05;
            }

            if (xpressed){
                Constants.Spin.ShootSpeed -= 0.05;
            }*/

            if (rtPressed){
                /*if (angler.getAnglePos() <= 2.0){
                    ShootSpeed = 0.25;
                } else if (angler.getAnglePos() <= 3.7 && angler.getAnglePos() >= 2.0) {
                    ShootSpeed = 0.35;
                }else if (angler.getAnglePos() >= 3.7){
                    ShootSpeed = 0.5;
                }*/

                double ShootSpeed = AnglerShoot(angler.getAnglePos());
                double ShootReq = ShootReq(angler.getAnglePos());

                //double ShootSpeed = SmartDashboard.getNumber("Shoot Speed",   Constants.Spin.ShootSpeed);
                //ShootSpeed = Constants.Spin.ShootSpeed;
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;

                if (Constants.Spin.ClosedLoopShooter) {
                    shoot.setShooterRPS(Constants.Spin.TestTargetRPS, Constants.Spin.TestTargetRPS);
                } else {
                    shoot.OpenShootSpeed(ShootSpeed);
                }

                if(shoot.getLeftRPS() >= ShootReq){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                } else {
                    shoot.SpinSpeed(0);
                    shoot.roller(0);
                }
            } else if (joyLeftPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(-Rollerspeed);
            } else if (!rtPressed && !ltPressed){
                shoot.SpinSpeed(0);
                shoot.roller(0);

                if (Constants.Spin.ClosedLoopShooter) {
                    shoot.setShooterRPS(Constants.Spin.TestTargetRPS, Constants.Spin.TestTargetRPS);
                } else {
                    shoot.OpenShootSpeed(Constants.Spin.ShootSpeed);
                }
                
            }
        }

        
    }

}
