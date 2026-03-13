package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.SpinnerAndShooter;

public class AutoController {

    private intake Intake;
    private SpinnerAndShooter Shoot;

    public AutoController(intake Intake, SpinnerAndShooter Shoot) {
        this.Intake = Intake;
        this.Shoot = Shoot;
    }

    /*public Command Shoot(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Shoot.ShootSpeed(Constants.Spin.ShootSpeed), Shoot),
            new WaitCommand(0.5),
            new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot),
            new WaitCommand(2),
            new InstantCommand(() -> Shoot.SpinSpeed(0), Shoot)
        );
    }*/

    /*public Command IntakeExtend(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake)
        );
    }*/


    
}