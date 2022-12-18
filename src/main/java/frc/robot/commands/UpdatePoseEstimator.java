// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class UpdatePoseEstimator extends CommandBase {

  Drivetrain subDrivetrain;
  Vision subVision;

  public UpdatePoseEstimator(Drivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;
    // does not require drivetrain
    addRequirements(subVision);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    subDrivetrain.updatePoseEstimator();

    PhotonPipelineResult result = subVision.photonCamera.getLatestResult();
    PhotonTrackedTarget[] filteredResult = subVision.getValidTargets(result);

    if (prefVision.useVision.getValue()) {
      if (result.getTargets().size() > 0 && filteredResult != null && filteredResult.length > 0) {
        subDrivetrain.addVisionMeasurement(subVision.calculatePoseFromTargets(filteredResult),
            result.getTimestampSeconds());
      }
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
