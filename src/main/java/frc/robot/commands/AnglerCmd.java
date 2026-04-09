package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Swerve;

public class AnglerCmd extends Command{

    private final Angler Angler;
    private final XboxController Operator;
    private final XboxController Driver;
    private final Swerve swerve;

    private double AnglePos;

    private double speed;

    public AnglerCmd(Angler Angler, XboxController Operator, XboxController Driver, Swerve swerve) {
        this.Angler = Angler;
        addRequirements(this.Angler);

        this.Operator = Operator;
        this.Driver = Driver;
        this.swerve = swerve;
        AnglePos = Angler.getAnglePos(); 

    }

    @Override
    public void initialize() {
    }

    @Override 
        public void execute() {
         if (DriverStation.isTeleop()) {

            boolean rtPressed = Operator.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            //boolean apressed = Operator.getAButtonPressed();
            boolean rtpressed = Driver.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean clear = !(swerve.getPose().getX() < 5.5 && swerve.getPose().getX() > 4) && !(swerve.getPose().getX() < 12.5 && swerve.getPose().getX() > 11.25);
            boolean trench = (swerve.getPose().getX() < 5.5 && swerve.getPose().getX() > 4) || (swerve.getPose().getX() < 12.5 && swerve.getPose().getX() > 11.25);

            if (rtPressed && !rtpressed){
                    Angler.updateFromTrackedAprilTag();
            } else if (rtPressed && rtpressed){
                    Angler.setAnglePosition(Constants.Angler.Passing);
            } else {
                Angler.setAnglePosition(0);
            }

            // if (xpressed){
            //     if (AnglePos <= 5.5){
            //         Constants.Angler.TestAngle += 0.5;
            //         AnglePos = Constants.Angler.TestAngle;
            //         Angler.setAnglePosition(AnglePos);
            //     }
            // }
            
            // if (apressed){
            //     if (AnglePos >= 0.5){
            //         Constants.Angler.TestAngle -= 0.5;
            //         AnglePos = Constants.Angler.TestAngle;
            //         Angler.setAnglePosition(AnglePos);
            //     }
            // }

            /*  if (ypressed){
                AnglePos = Constants.Angler.MaxAngle;
                Angler.setAnglePosition(AnglePos);
            }

            if (apressed) {
                AnglePos = Constants.Angler.TestAngle;
                Angler.setAnglePosition(AnglePos);
            }

            if (bpressed){
                AnglePos = Constants.Angler.MinAngle;
                Angler.setAnglePosition(AnglePos);
            }*/

         }

        }
}
