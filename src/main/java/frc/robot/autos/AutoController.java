package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.SpinnerAndShooter;
import frc.robot.subsystems.Angler;

public class AutoController {

    private intake Intake;
    private SpinnerAndShooter Shoot;
    private Angler angler;

    double ShootSpeed;

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

    public AutoController(intake Intake, SpinnerAndShooter Shoot, Angler angler) {
        this.Intake = Intake;
        this.Shoot = Shoot;
        this.angler = angler;
    }

    public Command Shoot(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> ShootSpeed = AnglerShoot(angler.getAnglePos())),
            // TODO: Change this to use the new shooter functionality instead of the old one, but for now we will just leave the old one commented since we haven't tested the new one yet
            //new InstantCommand(() -> Shoot.ShootSpeed(ShootSpeed), Shoot),
            new WaitCommand(0.5),
            new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot),
            new WaitCommand(2),
            new InstantCommand(() -> Shoot.SpinSpeed(Constants.Spin.CoastSpeed), Shoot)
        );
    }

    public Command Intake(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakeSpeed(Constants.Intake.IntakeSpeed), Intake),
            new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot),
            new WaitCommand(2),
            new InstantCommand(() -> Intake.setIntakeSpeed(0), Intake)
        );
    }

    public Command IntakeExtend(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake)
        );
    }


    
}