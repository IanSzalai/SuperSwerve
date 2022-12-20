// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constController;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class FlickStick extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;
  boolean fieldRelative;
  boolean isDriveOpenLoop;

  Rotation2d lastAngle;

  public FlickStick(
      Drivetrain subDrivetrain,
      SN_F310Gamepad conDriver,
      boolean fieldRelative,
      boolean isDriveOpenLoop) {
    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;
    this.fieldRelative = fieldRelative;
    this.isDriveOpenLoop = isDriveOpenLoop;

    lastAngle = subDrivetrain.getPose().getRotation();

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    lastAngle = subDrivetrain.getPose().getRotation();
  }

  @Override
  public void execute() {

    // get joystick inputs
    double xStick = conDriver.getAxisLSX();
    double yStick = conDriver.getAxisLSY();
    double rStickX = conDriver.getAxisRSX();
    double rStickY = conDriver.getAxisRSY();

    // apply deadband
    xStick = MathUtil.applyDeadband(xStick, prefDrivetrain.controllerDeadband.getValue());
    yStick = MathUtil.applyDeadband(yStick, prefDrivetrain.controllerDeadband.getValue());
    rStickX = MathUtil.applyDeadband(rStickX, prefDrivetrain.absSteerControllerDeadband.getValue());
    rStickY = MathUtil.applyDeadband(rStickY, prefDrivetrain.absSteerControllerDeadband.getValue());

    // apply slew rate limiter
    xStick = subDrivetrain.driveXSlewRateLimiter.calculate(xStick);
    yStick = subDrivetrain.driveYSlewRateLimiter.calculate(yStick);
    // can't slew rate limit the steer sticks

    // scale to proper units
    xStick *= Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    yStick *= Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());

    // create rotation from joysticks

    Rotation2d rotation = new Rotation2d();

    if (rStickX != 0 || rStickY != 0) {
      rotation = new Rotation2d(
          Math.atan2(rStickY, rStickX) + Units.degreesToRadians(constController.ABSOLUTE_STEER_OFFSET));
    } else {
      rotation = lastAngle;
    }

    SmartDashboard.putNumber("controller rotation angle", rotation.getDegrees());

    // create velocity vector
    Pose2d velocity = new Pose2d(xStick, yStick, rotation);

    subDrivetrain.driveAlignAngle(velocity, isDriveOpenLoop);

    lastAngle = rotation;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
