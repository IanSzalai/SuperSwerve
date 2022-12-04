package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.DriveAbsRotation;
import frc.robot.commands.DriveSimple;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.getInRangeOfTag;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Vision subVision = new Vision();

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new DriveSimple(subDrivetrain, conDriver, true, true));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_X.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
    conDriver.btn_Y.whenPressed(new InstantCommand(() -> subDrivetrain.configure()));
    conDriver.btn_A.whenPressed(new InstantCommand(() -> subDrivetrain.resetPose(new Pose2d())));
    conDriver.btn_B.whileHeld(new getInRangeOfTag(subVision, subDrivetrain));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
