package frc.robot;

import java.util.HashMap;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.commands.DriveSimple;
import frc.robot.commands.UpdatePoseEstimator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Vision subVision = new Vision();

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new DriveSimple(subDrivetrain, conDriver, true, true));

    subVision.setDefaultCommand(new UpdatePoseEstimator(subDrivetrain, subVision));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_X.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
    conDriver.btn_Y.whenPressed(new InstantCommand(() -> subDrivetrain.configure()));
    conDriver.btn_A.whenPressed(new InstantCommand(() -> subDrivetrain.resetPose(new Pose2d())));
  }

  public Command getAutonomousCommand() {

    PathPlannerTrajectory SCurvePath = PathPlanner.loadPath("SCurve",
        new PathConstraints(prefDrivetrain.transMaxSpeedFeet.getValue(), prefDrivetrain.transMaxAccelFeet.getValue()));
    PathPlannerTrajectory ThreeMeter = PathPlanner.loadPath("3Meter",
        new PathConstraints(prefDrivetrain.transMaxSpeedFeet.getValue(), prefDrivetrain.transMaxAccelFeet.getValue()));

    return subDrivetrain.autoBuilder.fullAuto(SCurvePath)
        .andThen(new InstantCommand(() -> subDrivetrain.neutralOutputs(), subDrivetrain));
  }
}
