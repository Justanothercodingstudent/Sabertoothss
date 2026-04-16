package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.Map;

public final class Autos {
  private Autos() {}

  public static void registerNamedCommands(Swerve swerve, AutoController autoController) {
    NamedCommands.clearAll();
    NamedCommands.registerCommands(
        Map.of(
            "resetHeading", new InstantCommand(swerve::zeroHeading, swerve),
            "Intake Up", autoController.IntakeUp(),
            "Intake Extend", autoController.IntakeExtend(),
            "Intake", autoController.Intake(),
            "Intake Stop", autoController.IntakeStop()
            // "Intake Jig", autoController.IntakeJig()
            ));

    NamedCommands.registerCommands(
        Map.of(
            "Shoot And Score", autoController.ShootAndScore(),
            "flip heading", swerve.flipHeading()));
  }

  public static SendableChooser<Command> buildChooser() {
    if (AutoBuilder.isConfigured()) {
      return AutoBuilder.buildAutoChooser(Constants.AutoConstants.defaultAutoName);
    }

    DriverStation.reportError(
        "PathPlanner AutoBuilder is not configured. Falling back to do-nothing auto.", false);
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", Commands.none());
    return chooser;
  }

  public static Pose2d getStartingPose(Command autoCommand) {
    if (!(autoCommand instanceof PathPlannerAuto pathPlannerAuto)) {
      return null;
    }

    Pose2d startingPose = pathPlannerAuto.getStartingPose();
    if (startingPose == null) {
      return null;
    }

    return Constants.TeamDependentFactors.isRedTeam
        ? FlippingUtil.flipFieldPose(startingPose)
        : startingPose;
  }
  // This is a lambda that will be called to register the named commands when the AutoBuilder is
  // configured.
  // You can put any code here that you want to run when the AutoBuilder is ready, such as
  // registering additional commands or performing setup tasks.)
}
