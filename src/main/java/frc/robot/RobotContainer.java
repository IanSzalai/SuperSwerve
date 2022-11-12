package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER);
  private final Drivetrain subDrivetrain = new Drivetrain();

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new Drive(subDrivetrain, conDriver, false, true));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    conDriver.btn_X.whenPressed(new InstantCommand(() -> subDrivetrain.zeroGyroYaw()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
