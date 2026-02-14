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

    private double speed;

    public climberCmd(climber climber, XboxController xbox){
        this.climber = climber;
        addRequirements(this.climber);

        this.xbox = xbox;

    }

     @Override
    public void initialize() {
    }

     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;
            boolean rtPressed = xbox.getRightTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            SmartDashboard.putBoolean("Right Trigger Button Pressed", ltPressed); // Debugging

            if (ltPressed){
                climber.setClimberPosition(Constants.climber.maxclimb);
            } else if (rtPressed){
                climber.setClimberPosition(Constants.climber.minClimb);
            }
        }

        
    }
}
