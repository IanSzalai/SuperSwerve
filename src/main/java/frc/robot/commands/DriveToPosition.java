// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToPosition extends CommandBase {

  Drivetrain subDrivetrain;

  public DriveToPosition(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    Pose2d goalPose = new Pose2d(0, 0, new Rotation2d(0));

    subDrivetrain.xTransPIDController.setGoal(new TrapezoidProfile.State(goalPose.getX(), 0));
    subDrivetrain.yTransPIDController.setGoal(new TrapezoidProfile.State(goalPose.getY(), 0));
    subDrivetrain.thetaPIDController.setGoal(new TrapezoidProfile.State(goalPose.getRotation().getRadians(), 0));

    double xVelocity = subDrivetrain.xTransPIDController.calculate(subDrivetrain.getPose().getX());
    double yVelocity = subDrivetrain.xTransPIDController.calculate(subDrivetrain.getPose().getY());
    double thetaVelocity = subDrivetrain.xTransPIDController
        .calculate(subDrivetrain.getPose().getRotation().getRadians());

    Pose2d velocityPose = new Pose2d(xVelocity, yVelocity, new Rotation2d(thetaVelocity));

    subDrivetrain.driveAlignAngle(velocityPose, true);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}