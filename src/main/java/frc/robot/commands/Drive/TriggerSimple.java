package frc.robot.commands.Drive;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class TriggerSimple extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;
  boolean fieldRelative;
  boolean isDriveOpenLoop;

  public TriggerSimple(
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

    // get controller inputs
    double xStick = conDriver.getAxisLSX();
    double yStick = conDriver.getAxisLSY();
    double rStick = -conDriver.getAxisRSX();
    double rTrigger = conDriver.getAxisRT();
    double lTrigger = conDriver.getAxisLT();

    // apply deadbands
    xStick = MathUtil.applyDeadband(xStick, prefDrivetrain.leftStickDeadband.getValue());
    yStick = MathUtil.applyDeadband(yStick, prefDrivetrain.leftStickDeadband.getValue());
    rStick = MathUtil.applyDeadband(rStick, prefDrivetrain.rightStickDeadband.getValue());
    rTrigger = MathUtil.applyDeadband(rTrigger, prefDrivetrain.rightTriggerDeadband.getValue());
    lTrigger = MathUtil.applyDeadband(lTrigger, prefDrivetrain.leftTriggerDeadband.getValue());

    // calculate components of translation
    double translationMagnitude = rTrigger - lTrigger;
    Rotation2d translationDirection = new Rotation2d(xStick, yStick);

    // apply slew rate limiter
    double rStickSlewed = subDrivetrain.steerSlewRateLimiter.calculate(rStick);

    // scale values to proper units
    double translationVelocity = translationMagnitude
        * Units.feetToMeters(prefDrivetrain.maxChassisSpeedFeet.getValue());
    double rotationVelocity = rStickSlewed
        * Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue());

    // create velocity pose
    Pose2d velocity = new Pose2d(
        new Translation2d(translationVelocity, translationDirection),
        new Rotation2d(rotationVelocity));

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
