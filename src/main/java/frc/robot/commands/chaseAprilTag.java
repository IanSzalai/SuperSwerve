// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class ChaseAprilTag extends CommandBase {
  Vision subVision;
  Drivetrain subDrivetrain;

  public ChaseAprilTag(Vision subVision, Drivetrain subDrivetrain) {
    this.subVision = subVision;
    this.subDrivetrain = subDrivetrain;

    addRequirements(this.subDrivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    PhotonPipelineResult result = subVision.photonCamera.getLatestResult();
    // this may need Math.PI, depends on robot setup
    Transform3d desiredDistance = new Transform3d(
        new Translation3d(RobotPreferences.prefVision.goalDistToTag.getValue(), 0, 0), new Rotation3d(0, 0, 0));

    Pose2d goalPose = null;
    // Filtering is done in goalPose to include an ID filter
    if (result != null && result.hasTargets()) {
      goalPose = subVision.getTargetRelativeGoalPose(
          RobotPreferences.prefVision.chasingTagID.getValue(),
          desiredDistance,
          subDrivetrain.getPose(),
          result);
    }

    if (goalPose != null && !(goalPose.equals(new Pose2d()))) {
      // subDrivetrain.driveToPosition(goalPose);
      SmartDashboard.putString(".goal pose", goalPose.toString());
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
