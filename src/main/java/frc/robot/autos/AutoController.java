package frc.robot.autos;

import java.time.Instant;

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

    // public Command Shoot(){
    //     return new SequentialCommandGroup(
    //         new InstantCommand(() -> angler.updateFromTrackedAprilTag(), angler),
    //         new WaitCommand(2),
    //         new InstantCommand(() -> Shoot.setShooterRPS(AnglerShoot(angler.getAnglePos()), AnglerShoot(angler.getAnglePos())), Shoot)
            
    //     );
    // }

    // public Command ShootStop(){
    //     return new ParallelCommandGroup(
    //         new InstantCommand(() -> angler.setAnglePosition(0), angler),
    //         new InstantCommand(() -> Shoot.OpenShootSpeed(0), Shoot)
    //     );
    // }

    public Command Intake(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakeSpeed(Constants.Intake.IntakeSpeed), Intake)
        );
    }

    public Command IntakeStop(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakeSpeed(0), Intake)
        );
    }

    public Command IntakeExtend(){
        return new ParallelCommandGroup(
            new InstantCommand(() -> Intake.setIntakePosition(Constants.Intake.maxExtend), Intake)
        );
    }

    // public Command Rollers(){
    //     return new ParallelCommandGroup(
    //         new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot)
    //     );
    // }

    //  public Command RollersStop(){
    //      return new ParallelCommandGroup(
    //          new InstantCommand(() -> Shoot.roller(0), Shoot)
    //      );
    // }

    // public Command Uptake(){
    //     return new ParallelCommandGroup(
    //         new InstantCommand(() -> Shoot.SpinSpeed(Constants.Spin.SpinSpeed), Shoot)
    //     );
    // }

    // public Command UptakeStop(){
    //     return new ParallelCommandGroup(
    //         new InstantCommand(() -> Shoot.SpinSpeed(0), Shoot)
    //     );
    // }

    public Command ShootAndScore(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> angler.updateFromTrackedAprilTag(), angler),
            new WaitCommand(1),
            new InstantCommand(() -> Shoot.setShooterRPS(AnglerShoot(angler.getAnglePos()), AnglerShoot(angler.getAnglePos())), Shoot),
            new WaitCommand(0.5),
            new InstantCommand(() -> Shoot.SpinSpeed(Constants.Spin.SpinSpeed), Shoot),
            new InstantCommand(() -> Shoot.roller(Constants.Spin.Rollerspeed), Shoot),
            new WaitCommand(4),
            new InstantCommand(() -> angler.setAnglePosition(0), angler),
            new InstantCommand(() -> Shoot.OpenShootSpeed(0), Shoot),
            new InstantCommand(() -> Shoot.roller(0), Shoot),
            new InstantCommand(() -> Shoot.SpinSpeed(0), Shoot)
        );
    }

    /*public Command FIRE(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> IntakeExtend()),
            new InstantCommand(() -> Shoot()),
            new WaitCommand(0.5),
            new InstantCommand(() -> Uptake()),
            new InstantCommand(() -> Rollers()),
            new WaitCommand(1),
            new InstantCommand(() -> ShootStop()),
            new InstantCommand(() -> RollersStop()),
            new InstantCommand(() -> UptakeStop())
        );
    }*/
}