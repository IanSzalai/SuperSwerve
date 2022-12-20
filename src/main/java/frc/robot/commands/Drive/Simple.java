package frc.robot.commands.Drive;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Simple extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;
  boolean fieldRelative;
  boolean isDriveOpenLoop;

  public Simple(
      Drivetrain subDrivetrain,
      SN_F310Gamepad conDriver,
      boolean fieldRelative,
      boolean isDriveOpenLoop) {
    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;
    this.fieldRelative = fieldRelative;
    this.isDriveOpenLoop = isDriveOpenLoop;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // get joystick inputs
    double xStick = conDriver.getAxisLSX();
    double yStick = conDriver.getAxisLSY();
    double rStick = -conDriver.getAxisRSX();

    xStick = MathUtil.applyDeadband(xStick, prefDrivetrain.leftStickDeadband.getValue());
    yStick = MathUtil.applyDeadband(yStick, prefDrivetrain.leftStickDeadband.getValue());
    rStick = MathUtil.applyDeadband(rStick, prefDrivetrain.rightStickDeadband.getValue());

    SmartDashboard.putNumber("x Stick", xStick);
    SmartDashboard.putNumber("y Stick", yStick);
    SmartDashboard.putNumber("r Stick", rStick);

    // scale joystick inputs to proper units
    double xVelocity = xStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double yVelocity = xStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double rVelocity = xStick * Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue());

    SmartDashboard.putNumber("x Velocity MPS", xVelocity);
    SmartDashboard.putNumber("y Velocity MPS", yVelocity);
    SmartDashboard.putNumber("r Velocity RPS", rVelocity);

    SmartDashboard.putNumber("x Velocity FPS", Units.metersToFeet(xVelocity));
    SmartDashboard.putNumber("y Velocity FPS", Units.metersToFeet(yVelocity));
    SmartDashboard.putNumber("r Velocity DPS", Units.radiansToDegrees(rVelocity));

    // create velocity pose with scaled joystick inputs
    Pose2d velocity = new Pose2d(
        xVelocity, yVelocity,
        new Rotation2d(rVelocity));

    subDrivetrain.drive(velocity, fieldRelative, isDriveOpenLoop, true);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
