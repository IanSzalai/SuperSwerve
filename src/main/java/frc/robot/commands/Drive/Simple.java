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

/**
 * Drive the robot using simple control. Left stick controls the direction and
 * speed of translation, right stick controls the rate and direction of
 * rotation.
 * 
 * sticks/buttons requires:
 * left stick (translation)
 * right stick (rotation)
 */
public class Simple extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;

  public Simple(Drivetrain subDrivetrain, SN_F310Gamepad conDriver) {

    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;

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

    SmartDashboard.putNumber(".raw stick x", xStick);
    SmartDashboard.putNumber(".raw stick y", yStick);
    SmartDashboard.putNumber(".raw stick r", rStick);
    SmartDashboard.putNumber(".raw stick ry", conDriver.getAxisRSY());

    xStick = MathUtil.applyDeadband(xStick, prefDrivetrain.leftStickDeadband.getValue());
    yStick = MathUtil.applyDeadband(yStick, prefDrivetrain.leftStickDeadband.getValue());
    rStick = MathUtil.applyDeadband(rStick, prefDrivetrain.rightStickDeadband.getValue());

    SmartDashboard.putNumber(".stick x", xStick);
    SmartDashboard.putNumber(".stick y", yStick);
    SmartDashboard.putNumber(".stick r", rStick);

    // scale joystick inputs to proper units
    double xVelocity = xStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double yVelocity = yStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double rVelocity = rStick * Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue());

    SmartDashboard.putNumber(".velocity x", xVelocity);
    SmartDashboard.putNumber(".velocity y", yVelocity);
    SmartDashboard.putNumber(".velocity r", rVelocity);

    // create velocity pose with scaled joystick inputs
    Pose2d velocity = new Pose2d(
        xVelocity, yVelocity,
        new Rotation2d(rVelocity));

    subDrivetrain.drive(velocity);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
