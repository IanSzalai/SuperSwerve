package frc.robot.commands;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {

  Drivetrain subDrivetrain;
  SN_F310Gamepad conDriver;
  boolean fieldRelative;
  boolean isOpenLoop;

  public Drive(Drivetrain subDrivetrain, SN_F310Gamepad conDriver, boolean fieldRelative, boolean isOpenLoop) {
    this.subDrivetrain = subDrivetrain;
    this.conDriver = conDriver;
    this.fieldRelative = fieldRelative;
    this.isOpenLoop = isOpenLoop;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // get joystick inputs
    double xVelocity = conDriver.getAxisLSX();
    double yVelocity = conDriver.getAxisLSY();
    double rVelocity = conDriver.getAxisRSX();

    // apply slew rate limiter
    xVelocity = subDrivetrain.driveSlewRateLimiter.calculate(xVelocity);
    yVelocity = subDrivetrain.driveSlewRateLimiter.calculate(yVelocity);
    rVelocity = subDrivetrain.steerSlewRateLimiter.calculate(rVelocity);

    // scale slewed joystick inputs to proper units
    xVelocity *= Units.feetToMeters(prefDrivetrain.maxSpeedFPS.getValue());
    yVelocity *= Units.feetToMeters(prefDrivetrain.maxSpeedFPS.getValue());
    rVelocity *= Units.degreesToRadians(prefDrivetrain.maxRotationDPS.getValue());

    // create velocity pose with scaled, slewed joystick inputs
    Pose2d velocity = new Pose2d(
        xVelocity, yVelocity,
        new Rotation2d(rVelocity));

    subDrivetrain.drive(velocity, fieldRelative, isOpenLoop);

  }

  @Override
  public void end(boolean interrupted) {
    subDrivetrain.drive(new Pose2d(0, 0, new Rotation2d(0)), fieldRelative, isOpenLoop);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
