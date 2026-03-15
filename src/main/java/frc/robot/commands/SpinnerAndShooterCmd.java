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

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean ltPressed = xbox.getLeftTriggerAxis()  > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean joyLeftPressed = xbox.getLeftStickButtonPressed();

            SmartDashboard.putBoolean("Right Trigger Button Pressed", rtPressed); // Debugging

            if (rtPressed){
                if (angler.getAnglePos() <= 2.0){
                    ShootSpeed = 0.25;
                } else if (angler.getAnglePos() <= 3.7 && angler.getAnglePos() >= 2.0) {
                    ShootSpeed = 0.35;
                }else if (angler.getAnglePos() >= 3.7){
                    ShootSpeed = 0.5;
                }

                //double ShootSpeed = SmartDashboard.getNumber("Shoot Speed",   Constants.Spin.ShootSpeed);
                SpinSpeed = Constants.Spin.SpinSpeed;
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.ShootSpeed(ShootSpeed);
                if(shoot.FrontLeftRPM() <= -15){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                } else if(shoot.FrontLeftRPM() <= -35){
                    shoot.roller(Rollerspeed);
                    shoot.SpinSpeed(SpinSpeed);
                }
            } else if (joyLeftPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                shoot.roller(-Rollerspeed);
            } else if (!rtPressed & !ltPressed){
                shoot.SpinSpeed(0);
                shoot.roller(0);
                shoot.ShootSpeed(Constants.Spin.CoastSpeed);
                
            }
        }

        
    }

}
