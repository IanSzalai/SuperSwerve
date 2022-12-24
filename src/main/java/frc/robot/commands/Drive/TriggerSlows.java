package frc.robot.commands.Drive;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

/**
 * Drive the robot using triggers to control translational speed, and the left
 * stick to control the translational direction. Right stick controls the rate
 * and direction of the translation like simple.
 * 
 * sticks/buttons requires:
 * left stick (translation)
 * right stick (rotation)
 * left trigger (translation)
 * right trigger (translation)
 */
public class TriggerSlows extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;

  Rotation2d lastTranslationDirection;

  public TriggerSlows(Drivetrain subDrivetrain, SN_F310Gamepad conDriver) {

    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;

    lastTranslationDirection = new Rotation2d();

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // get controller inputs
    double xStick = conDriver.getAxisLSX();
    double yStick = conDriver.getAxisLSY();
    double rStick = -conDriver.getAxisRSX();
    double rTrigger = conDriver.getAxisRT();

    // apply deadbands
    xStick = MathUtil.applyDeadband(xStick, prefDrivetrain.leftStickDeadband.getValue());
    yStick = MathUtil.applyDeadband(yStick, prefDrivetrain.leftStickDeadband.getValue());
    rStick = MathUtil.applyDeadband(rStick, prefDrivetrain.rightStickDeadband.getValue());
    rTrigger = MathUtil.applyDeadband(rTrigger, prefDrivetrain.rightTriggerDeadband.getValue());

    // calculate components of translation
    double translationMagnitude = new Translation2d(xStick, yStick).getNorm()
        * SN_Math.interpolate(rTrigger, 0, 1, 1, .2);
    Rotation2d translationDirection = new Rotation2d();
    if (xStick != 0 || yStick != 0) {
      translationDirection = new Rotation2d(xStick, yStick);
    } else {
      translationDirection = lastTranslationDirection;
      translationMagnitude = 0;
    }
    lastTranslationDirection = translationDirection;
    // scale values to proper units
    double translationVelocity = translationMagnitude
        * Units.feetToMeters(prefDrivetrain.teleMaxSpeedFeet.getValue());
    double rotationVelocity = rStick
        * Units.degreesToRadians(prefDrivetrain.teleMaxRotSpeedDegrees.getValue());

    // create velocity pose
    Pose2d velocity = new Pose2d(
        new Translation2d(translationVelocity, translationDirection),
        new Rotation2d(rotationVelocity));

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
