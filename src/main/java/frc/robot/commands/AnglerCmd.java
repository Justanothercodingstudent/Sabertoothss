package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Angler;

public class AnglerCmd extends Command{

    private final Angler Angler;
    private final XboxController xbox;

    private double AnglePos;

    private double speed;

    public AnglerCmd(Angler Angler, XboxController xbox){
        this.Angler = Angler;
        addRequirements(this.Angler);

        this.xbox = xbox;

        AnglePos = Angler.getAnglePos(); 

    }

    @Override
    public void initialize() {
    }

    @Override 
        public void execute() {
         if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean apressed = xbox.getAButton();
            boolean bpressed = xbox.getBButton();


           /*  if (ltPressed){
                speed = Constants.Angler.AngleSpeed;
                Angler.AngleSpeed(speed);
            }else if (rtPressed){
                speed = Constants.Angler.AngleSpeed;
                Angler.AngleSpeed(-speed);
            } else {
                Angler.AngleSpeed(0);
            }*/

            if (apressed) {
                AnglePos = Constants.Angler.TestAngle;
                Angler.setAnglePosition(AnglePos);
            }

            if (bpressed){
                AnglePos = Constants.Angler.MinAngle;
                Angler.setAnglePosition(AnglePos);
            }

         }

        }
}
