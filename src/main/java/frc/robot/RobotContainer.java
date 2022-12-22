package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.commands.UpdatePoseEstimator;
import frc.robot.commands.Drive.Simple;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Vision subVision = new Vision();

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new Simple(subDrivetrain, conDriver));

    subVision.setDefaultCommand(new UpdatePoseEstimator(subDrivetrain, subVision));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_LBump.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
    conDriver.btn_RBump.whenPressed(new InstantCommand(() -> subDrivetrain.configure()));
    conDriver.btn_Start.whenPressed(new InstantCommand(() -> subDrivetrain.resetPose(new Pose2d())));
    conDriver.btn_Back.whenPressed(new InstantCommand(() -> subDrivetrain.toggleFieldRelative()));
  }

  public Command getAutonomousCommand() {

    PathPlannerTrajectory DiamondPath = PathPlanner.loadPath("Diamond",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));
    PathPlannerTrajectory Figure8 = PathPlanner.loadPath("Figure8",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));
    PathPlannerTrajectory SCurvePath = PathPlanner.loadPath("SCurve",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));
    PathPlannerTrajectory ThreeMeter = PathPlanner.loadPath("ThreeMeter",
        new PathConstraints(
            Units.feetToMeters(prefDrivetrain.autoMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.autoMaxAccelFeet.getValue())));

    return subDrivetrain.autoBuilder.fullAuto(Figure8)
        .andThen(new InstantCommand(() -> subDrivetrain.neutralOutputs(), subDrivetrain));
  }
}
