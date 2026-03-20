package frc.robot.autos;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public final class Autos {
    private Autos() {}

    public static void registerNamedCommands(Swerve swerve, AutoController autoController) {
        NamedCommands.clearAll();
        NamedCommands.registerCommands(Map.of(
            "resetHeading", new InstantCommand(swerve::zeroHeading, swerve),
            "Shoot", autoController.Shoot(),
            "Shoot Stop", autoController.ShootStop(),
            "Intake Extend", autoController.IntakeExtend(),
            "Intake", autoController.Intake(),
            "Intake Stop", autoController.IntakeStop(),
            "Rollers", autoController.Rollers(),
            "Rollers Stop", autoController.RollersStop(),
            "Uptake", autoController.Uptake(),
            "Uptake Stop", autoController.UptakeStop()
        ));
    }

    public static SendableChooser<Command> buildChooser() {
        if (AutoBuilder.isConfigured()) {
            return AutoBuilder.buildAutoChooser(Constants.AutoConstants.defaultAutoName);
        }

        DriverStation.reportError("PathPlanner AutoBuilder is not configured. Falling back to do-nothing auto.", false);
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());
        return chooser;
    }
}
