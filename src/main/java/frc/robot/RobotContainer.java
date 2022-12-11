package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.constVision.PoseEstimationType;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.DriveAbsRotation;
import frc.robot.commands.UpdatePoseEstimator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Vision subVision = new Vision();

  public static PoseEstimationType poseEstimationType;

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new DriveAbsRotation(subDrivetrain, conDriver, true, true));

    new UpdatePoseEstimator(subDrivetrain, subVision).perpetually();

    poseEstimationType = PoseEstimationType.GYRO_AND_VISION;

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_X.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
    conDriver.btn_Y.whenPressed(new InstantCommand(() -> subDrivetrain.configure()));
    conDriver.btn_A.whenPressed(new InstantCommand(() -> subDrivetrain.resetPose(new Pose2d())));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
