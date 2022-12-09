// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.constVision.PoseEstimationType;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class UpdatePoseEstimator extends CommandBase {

  Drivetrain subDrivetrain;
  Vision subVision;

  public UpdatePoseEstimator(Drivetrain subDrivetrain, Vision subVision) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;
    // does not require drivetrain
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    PhotonPipelineResult result = subVision.photonCamera.getLatestResult();
    
    switch (RobotContainer.poseEstimationType) {
      case GYRO_ONLY:
        subDrivetrain.updatePoseEstimator();  
        break;
      case VISION_ONLY:
        if(subVision.isPoseValid(result)){
          subDrivetrain.addVisionMeasurement(subVision.getVisionPose(result), result.getTimestampSeconds());
        }
        break;
      case GYRO_AND_VISION:
        subDrivetrain.updatePoseEstimator(); 
        if(subVision.isPoseValid(result)){
          subDrivetrain.addVisionMeasurement(subVision.getVisionPose(result), result.getTimestampSeconds());
        }
        break;
      case NONE:
        break;
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
