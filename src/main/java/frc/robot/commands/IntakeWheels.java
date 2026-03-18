package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeWheels extends Command{

    private final intake Intake;
    private final XboxController xbox;
    private final SpinnerAndShooter Spin;

    private double speed;
    private double Rollerspeed;

    public IntakeWheels(intake Intake, XboxController xbox, SpinnerAndShooter Spin){
        this.Intake = Intake;
        addRequirements(this.Intake);

        this.xbox = xbox;
        this.Spin = Spin;
    }
    
     @Override 
    public void execute() {
        if (DriverStation.isTeleop()) {

            boolean ltPressed = xbox.getLeftTriggerAxis() > Constants.OperatorConstants.TRIGGER_THRESHOLD;

            if (ltPressed){
                speed = Constants.Intake.IntakeSpeed;
                Intake.setIntakeSpeed(speed);
            }else{
                Intake.setIntakeSpeed(0);
            }

            if (ltPressed){
                Rollerspeed = Constants.Spin.Rollerspeed;
                Intake.roller(Rollerspeed);
            } else if (Spin.getLeftRPS() <= 5 && Intake.getIntakeSpeed() <= 1) {
                Intake.roller(0);
            }

        }
    }
}
