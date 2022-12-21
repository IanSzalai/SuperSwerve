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

  SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      subDrivetrain::getPose,
      subDrivetrain::resetPose,
      Constants.SWERVE_KINEMATICS,
      new PIDConstants(
          prefDrivetrain.transP.getValue(),
          prefDrivetrain.transI.getValue(),
          prefDrivetrain.transD.getValue()),
      new PIDConstants(
          prefDrivetrain.autoThetaP.getValue(),
          prefDrivetrain.autoThetaI.getValue(),
          prefDrivetrain.autoThetaD.getValue()),
      subDrivetrain::setModuleStates,
      new HashMap<>(),
      subDrivetrain);

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

    PathPlannerTrajectory SCurvePath = PathPlanner.loadPath("SCurvePath", new PathConstraints(2, 1));

    return autoBuilder.fullAuto(SCurvePath);
  }
}
