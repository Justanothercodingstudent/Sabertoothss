package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.intake;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class climberCmd extends Command{
    
    private final climber climber;
    private final XboxController xbox;

    private double ClimbPos;

    private double speed;

    public climberCmd(climber climber, XboxController xbox){
        this.climber = climber;
        addRequirements(this.climber);

        this.xbox = xbox;

        ClimbPos = climber.getclimberPos();

    }

     @Override
    public void initialize() {
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean lbPressed = xbox.getLeftBumperButtonPressed();
            boolean rbPressed = xbox.getRightBumperButtonPressed();
            boolean LtPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            SmartDashboard.putBoolean("Right Trigger Button Pressed", lbPressed); // Debugging

            /*if (lbPressed){
                ClimbPos = Constants.climber.maxClimb;
                climber.setClimberPosition(ClimbPos);
            } else if (rbPressed){
                ClimbPos = Constants.climber.minClimb;
                climber.setClimberPosition(ClimbPos);
            }*/

            
           //if (ltPressed){
           //     climber.ClimbSpeed(Constants.climber.ClimbSpeed);
          // } else if (rtPressed) {
           //     climber.ClimbSpeed(-Constants.climber.ClimbSpeed);
           //} else {
           //     climber.ClimbSpeed(0);
           //}
        } 
           

        
    }
}
