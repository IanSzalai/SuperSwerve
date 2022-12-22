package frc.robot.commands.Drive;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot using simple control, except that buttons will set the
 * rotational position. This position will be locked after a button press, until
 * another normal steering input (or different button).
 * 
 * sticks/buttons required:
 * left stick (translation)
 * right stick (rotation)
 * a (rotation)
 * b (rotation)
 * x (rotation)
 * y (rotation)
 */
public class Buttons extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;

  boolean isPositionSet;
  Rotation2d rotation;

  public Buttons(Drivetrain subDrivetrain, SN_F310Gamepad conDriver) {

    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;

    isPositionSet = false;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    isPositionSet = false;
    rotation = new Rotation2d();
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

    // scale joystick inputs to proper units
    double xVelocity = xStick * Units.feetToMeters(prefDrivetrain.teleMaxSpeedFeet.getValue());
    double yVelocity = yStick * Units.feetToMeters(prefDrivetrain.teleMaxSpeedFeet.getValue());
    double rVelocity = rStick * Units.degreesToRadians(prefDrivetrain.teleMaxRotSpeedDegrees.getValue());

    // create rotation using stick input
    /**
     * button layout:
     * ......Y......
     * ....X...B....
     * ......A......
     */

    // if there is a steer stick input ignore buttons
    if (Math.abs(rStick) > 0) {
      isPositionSet = false;
      rotation = new Rotation2d(rVelocity);
    }
    // if there is not a steer stick input, look at buttons and change the rotation
    // to match the angle of the button (0 degrees is facing right)
    else {
      if (conDriver.btn_A.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(180);
      }
      if (conDriver.btn_B.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(270);
      }
      if (conDriver.btn_X.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(90);
      }
      if (conDriver.btn_Y.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(0);
      }
    }
    // create a velocity pose with the x, y, and rotational components
    Pose2d velocity = new Pose2d(xVelocity, yVelocity, rotation);

    // if the steer stick wasn't used and a button was, drive with closed loop steer
    if (isPositionSet) {
      subDrivetrain.driveAlignAngle(velocity);
    }
    // if the steer stick was used, drive with open loop steer
    else {
      subDrivetrain.drive(velocity);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
