package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.Constants;

public class SpinnerAndShooterCmd extends Command{
    
    private final SpinnerAndShooter shoot;
    private final XboxController xbox;

    private double ShootSpeed;
    private double SpinSpeed;
    private double Rollerspeed;
    public SpinnerAndShooterCmd(SpinnerAndShooter shoot, XboxController xbox){
        this.shoot = shoot;
        addRequirements(this.shoot);

        this.xbox = xbox;

    }

     @Override
    public void initialize() {
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean ltPressed = xbox.getLeftTriggerAxis()  > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean joyLeftPressed = xbox.getLeftStickButtonPressed();

            SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            if (rtPressed){
                ShootSpeed = Constants.Spin.ShootSpeed;
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.ShootSpeed(ShootSpeed);
                if(shoot.TotalRPM() >= 70){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                }
            } else if (ltPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(Rollerspeed);
            } else if (joyLeftPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(-Rollerspeed);
            } else {
                shoot.ShootSpeed(0);
                shoot.SpinSpeed(0);
                shoot.roller(0);
            }
        }

        
    }

}
