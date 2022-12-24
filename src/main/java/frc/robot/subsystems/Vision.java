// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPreferences.prefVision;

public class Vision extends SubsystemBase {

  public PhotonCamera photonCamera;
  PhotonPipelineResult lastResult = null;

  public Vision() {
    // Update with your photon camera's name
    photonCamera = new PhotonCamera("limelight");
  }

  // Filters out any targets with too high of a pose ambiguity (or other filters)
  public PhotonTrackedTarget[] getValidTargets(PhotonPipelineResult result) {
    PhotonTrackedTarget[] filteredResult = null;
    if (!result.equals(lastResult) && result.hasTargets()) {
      filteredResult = result.getTargets().stream()
          .filter(t -> t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1).toArray(PhotonTrackedTarget[]::new);
    }
    return filteredResult;
  }

  // Get distance from all targets that you can see (getValidTargets) and then
  // calculate an average Pose3d from that.
  // Quaternions are used because they are continuous
  public Pose2d calculatePoseFromTargets(PhotonTrackedTarget[] filteredResult) {
    Translation3d totalTranslation = new Translation3d(0, 0, 0);
    Quaternion initialQuaternion = null;
    Quaternion newQuaternion = null;
    double totalW = 0;
    double totalX = 0;
    double totalY = 0;
    double totalZ = 0;

    Pose3d cameraPose = new Pose3d(prefVision.cameraXPositionInches.getValue(),
        prefVision.cameraYPositionInches.getValue(),
        prefVision.cameraZPositionInches.getValue(),
        new Rotation3d(prefVision.cameraRollDegrees.getValue(), prefVision.cameraPitchDegrees.getValue(),
            prefVision.cameraYawDegrees.getValue()));
    Transform3d cameraToRobot = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), cameraPose);

    if (filteredResult.length < 1) {
      return null;
    }

    for (PhotonTrackedTarget t : filteredResult) {
      Transform3d cameraToTag = t.getBestCameraToTarget();
      Pose3d targetPose = new Pose3d(2, 2, 0, new Rotation3d(0, 0, 0)); // This is where WPI Array code would go, for
                                                                        // now just only use 1 tag
      Pose3d cameraOnField = targetPose.transformBy(cameraToTag);
      Pose3d robotOnField = cameraOnField.transformBy(cameraToRobot);

      totalTranslation = totalTranslation.plus(robotOnField.getTranslation());

      if (initialQuaternion == null) {
        initialQuaternion = robotOnField.getRotation().getQuaternion();
        newQuaternion = initialQuaternion;
      } else {
        newQuaternion = robotOnField.getRotation().getQuaternion();
        // dot: ensure every value is within 180 degrees of init. quaternion. rahh
        // gimbal lock
        double dot = (initialQuaternion.getW() * newQuaternion.getW())
            + (initialQuaternion.getX() * newQuaternion.getX()) + (initialQuaternion.getY() * newQuaternion.getY())
            + (initialQuaternion.getZ() * newQuaternion.getZ());
        if (dot < 0) {
          newQuaternion = newQuaternion.inverse();
        }
      }
      totalW += newQuaternion.getW();
      totalX += newQuaternion.getX();
      totalY += newQuaternion.getY();
      totalZ += newQuaternion.getZ();
    }

    Translation3d averageTranslation = totalTranslation.div(filteredResult.length);
    double averageW = totalW / filteredResult.length;
    double averageX = totalX / filteredResult.length;
    double averageY = totalY / filteredResult.length;
    double averageZ = totalZ / filteredResult.length;

    Quaternion calculatedQuaternion = new Quaternion(averageW, averageX, averageY, averageZ);
    Pose3d calculatedPose = new Pose3d(averageTranslation, new Rotation3d(calculatedQuaternion));

    SmartDashboard.putNumber("robot calced pose x", calculatedPose.toPose2d().getX());
    SmartDashboard.putNumber("robot calced pose y", calculatedPose.toPose2d().getY());
    SmartDashboard.putNumber("robot calced pose rot", calculatedPose.toPose2d().getRotation().getDegrees());

    System.out.println(calculatedPose);

    return calculatedPose.toPose2d();
  }

  // Calculates a Field-relative goal pose relative to a visible target.
  public Pose2d getTargetRelativeGoalPose(
      double desiredTargetID,
      Transform3d desiredDistance,
      Pose2d robotPose,
      PhotonPipelineResult result) {
    Pose2d returnedPose = new Pose2d();

    Optional<PhotonTrackedTarget> filteredResult = result.getTargets().stream()
        .filter(t -> t.getPoseAmbiguity() <= .2)
        .filter(t -> t.getPoseAmbiguity() != -1)
        .filter(t -> t.getFiducialId() == desiredTargetID)
        .findFirst();

    if (filteredResult.isPresent()) {
      PhotonTrackedTarget target = filteredResult.get();

      Transform3d targetToCamera = target.getBestCameraToTarget();
      Transform3d aiogaoigioaw = targetToCamera.plus(desiredDistance);
      Pose3d targetCameraRelative = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)).transformBy(aiogaoigioaw);
      // this is done because photonvision people use a different coordinate system
      // and its very annoying

      // so this is where the target is relative to the camera (currently assuming the
      // camera is not offset at all, which it is, but shut up)
      Pose2d actualTargetPose = new Pose2d(targetCameraRelative.getY(),
          targetCameraRelative.getX(),
          new Rotation2d(targetCameraRelative.getRotation().getZ() - Units.degreesToRadians(180)));

      // now we make it field relative
      Transform2d targetToRobot = new Transform2d(new Pose2d(0, 0, new Rotation2d(0)), actualTargetPose);
      Pose2d targetFieldRelativePose = robotPose.transformBy(targetToRobot);

      returnedPose = targetFieldRelativePose;
    }

    SmartDashboard.putString(".desired pose", returnedPose.toString());
    return returnedPose;
  };

  @Override
  public void periodic() {
  }
}
