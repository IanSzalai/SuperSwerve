// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class calculatePoseFromVision extends CommandBase {
  Vision subVision;
  Drivetrain subDrivetrain;

  PhotonPipelineResult lastResult = null;
  
  public calculatePoseFromVision(Vision subVision, Drivetrain subDrivetrain) {
    this.subVision = subVision;
    this.subDrivetrain = subDrivetrain;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: create some a dictionary or something to store mutliple constant tag locations
    // WPI will do it for us, but I literally just need to know what "type of dictionary" to use to implement it now 
    // (I don't know enough Java) (and its 12:22 am) (also it would be cool)
    PhotonPipelineResult result = subVision.photonCamera.getLatestResult();

    if((!result.equals(lastResult)) && result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();
      lastResult = result;
      if(target.getPoseAmbiguity() <= 0.2 && target.getPoseAmbiguity() != -1){
        double timestamp = result.getTimestampSeconds();

        Pose3d cameraPose = new Pose3d(prefVision.cameraXPosition.getValue(), prefVision.cameraYPosition.getValue(),
          prefVision.cameraZPosition.getValue(),
          new Rotation3d(prefVision.cameraRoll.getValue(), prefVision.cameraPitch.getValue(),
              prefVision.cameraYaw.getValue()));
        Transform3d cameraToRobot = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), cameraPose);
       
        Transform3d cameraToTag = target.getBestCameraToTarget();
      
        Pose3d targetPose = new Pose3d(0,0,0, new Rotation3d(0,0,0));
        
        Pose3d cameraOnField = targetPose.transformBy(cameraToTag);
        Pose3d robotOnField = cameraOnField.transformBy(cameraToRobot);

        subDrivetrain.addVisionMeasurement(robotOnField.toPose2d(), timestamp);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
