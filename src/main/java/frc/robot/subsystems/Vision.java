// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences.prefVision;

public class Vision extends SubsystemBase {
  
  public PhotonCamera photonCamera;
  PhotonPipelineResult lastResult = null;

  public Vision() {
    // Update with your photon camera's name
    photonCamera = new PhotonCamera("limelight");
  }

  public boolean isPoseValid(PhotonPipelineResult result){
    boolean validity = false;
    if(!result.equals(lastResult) && result.hasTargets() && result.getBestTarget().getPoseAmbiguity() <= 0.2 && result.getBestTarget().getPoseAmbiguity() != -1){
      lastResult = result;
      validity = true;
    }
    return validity; 
  }

  public Pose2d getVisionPose(PhotonPipelineResult result){
    PhotonTrackedTarget target = result.getBestTarget();
      
    Pose3d cameraPose = new Pose3d(prefVision.cameraXPosition.getValue(), prefVision.cameraYPosition.getValue(),
      prefVision.cameraZPosition.getValue(),
      new Rotation3d(prefVision.cameraRoll.getValue(), prefVision.cameraPitch.getValue(),
      prefVision.cameraYaw.getValue()));
    Transform3d cameraToRobot = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), cameraPose);
      
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d(0,0,0));
        
    Pose3d cameraOnField = targetPose.transformBy(cameraToTag);
    Pose3d robotOnField = cameraOnField.transformBy(cameraToRobot);
      
    return robotOnField.toPose2d();
  }

  @Override
  public void periodic() {
  }
}
