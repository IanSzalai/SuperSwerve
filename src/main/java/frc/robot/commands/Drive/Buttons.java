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
 */
public class Buttons extends CommandBase {
  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;
  boolean fieldRelative;
  boolean isDriveOpenLoop;

  boolean isPositionSet;

  public Buttons(
      Drivetrain subDrivetrain,
      SN_F310Gamepad conDriver,
      boolean fieldRelative,
      boolean isDriveOpenLoop) {

    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;
    this.fieldRelative = fieldRelative;
    this.isDriveOpenLoop = isDriveOpenLoop;

    isPositionSet = false;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
    isPositionSet = false;
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
    double xVelocity = xStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double yVelocity = xStick * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double rVelocity = xStick * Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue());

    // create rotation using stick input
    Rotation2d rotation = new Rotation2d(rVelocity);
    /**
     * button layout:
     * ......Y......
     * ....X...B....
     * ......A......
     */

    // if there is a steer stick input ignore buttons
    if (rStick > 0) {
      isPositionSet = false;
    }
    // if there is not a steer stick input, look at buttons and change the rotation
    // to match the angle of the button (0 degrees is facing right)
    else {
      if (conDriver.btn_A.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(90);
      } else if (conDriver.btn_B.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(0);
      } else if (conDriver.btn_X.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(180);
      } else if (conDriver.btn_Y.get()) {
        isPositionSet = true;
        rotation = Rotation2d.fromDegrees(270);
      }
    }

    // create a velocity pose with the x, y, and rotational components
    Pose2d velocity = new Pose2d(xVelocity, yVelocity, rotation);

    // if the steer stick wasn't used and a button was, drive with closed loop steer
    if (isPositionSet) {
      subDrivetrain.driveAlignAngle(velocity, isDriveOpenLoop);
    }
    // if the steer stick was used, drive with open loop steer
    else {
      subDrivetrain.drive(velocity, fieldRelative, isDriveOpenLoop, false);
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
