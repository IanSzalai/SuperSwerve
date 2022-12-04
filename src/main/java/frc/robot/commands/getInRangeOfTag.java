// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// Note: This command assumes that you know where you are on the field, since every other position is relative to you

public class getInRangeOfTag extends CommandBase {

  Vision subVision;
  Drivetrain subDrivetrain;
  ChassisSpeeds speeds;

  Pose3d robotPose3d;
  Pose3d cameraPose;
  Transform3d cameraToRobot = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
      new Pose3d(prefVision.cameraXPosition.getValue(), prefVision.cameraYPosition.getValue(),
          prefVision.cameraZPosition.getValue(),
          new Rotation3d(prefVision.cameraRoll.getValue(), prefVision.cameraPitch.getValue(),
              prefVision.cameraYaw.getValue())));

  PhotonPipelineResult result;
  Optional<PhotonTrackedTarget> filteredResult;

  Transform3d camToTargetTrans;
  boolean hasDesiredTag;
  PhotonTrackedTarget target;
  PhotonTrackedTarget lastTarget = null;
  Pose3d targetPose;
  Pose2d velocityPose = new Pose2d();

  double desiredTagID = 33;

  // where we want to be relative to the tag. 1m in front of it, rotated by 180 so
  // that we're still looking at it
  Transform3d goalTranslation = new Transform3d(new Translation3d(1, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI));
  Pose3d goalPose;

  public getInRangeOfTag(Vision subVision, Drivetrain subDrivetrain) {
    this.subVision = subVision;
    this.subDrivetrain = subDrivetrain;

    addRequirements(this.subDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    result = subVision.photonCamera.getLatestResult();

    // Filter down to 1 tag position that is most accurate
    filteredResult = result.getTargets().stream()
        .filter(t -> t.getFiducialId() == desiredTagID)
        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        .findFirst();

    // If the filtered result exists, we see a new desired target
    if (filteredResult.isPresent()) {
      target = filteredResult.get();
      lastTarget = target;
      camToTargetTrans = target.getBestCameraToTarget();

      // we can do this because our z pose, our pitch, and our roll are constant
      robotPose3d = new Pose3d(subDrivetrain.getPose().getX(), subDrivetrain.getPose().getY(), 0,
          new Rotation3d(0, 0, subDrivetrain.getPose().getRotation().getRadians()));
      cameraPose = robotPose3d.transformBy(cameraToRobot);

      // where our target is relative to the field (not a constant in this scenario)
      targetPose = cameraPose.transformBy(camToTargetTrans);

      // define a goal position relative to the target
      goalPose = targetPose.transformBy(goalTranslation);

      // // go to goalPose
      // speeds = subDrivetrain.driveController.calculate(
      // subDrivetrain.getPose(),
      // goalPose.toPose2d(),
      // 1,
      // goalPose.getRotation().toRotation2d());

      // If we have seen a target (so speeds has a value)
      subDrivetrain.xTransPIDController.setGoal(new TrapezoidProfile.State(goalPose.getX(), 0));
      subDrivetrain.yTransPIDController.setGoal(new TrapezoidProfile.State(goalPose.getY(), 0));
      subDrivetrain.thetaPIDController
          .setGoal(new TrapezoidProfile.State(goalPose.getRotation().toRotation2d().getRadians(), 0));

      double xVelocity = subDrivetrain.xTransPIDController.calculate(subDrivetrain.getPose().getX());
      double yVelocity = subDrivetrain.yTransPIDController.calculate(subDrivetrain.getPose().getY());
      double thetaVelocity = subDrivetrain.thetaPIDController
          .calculate(subDrivetrain.getPose().getRotation().getRadians());

      velocityPose = new Pose2d(xVelocity, yVelocity, new Rotation2d(thetaVelocity));
    }
    if (lastTarget != null) {

      subDrivetrain.driveAlignAngle(velocityPose, true);
      SmartDashboard.putBoolean("lastTarget", true);
    } else {
      SmartDashboard.putBoolean("lastTarget", false);

      subDrivetrain.driveAlignAngle(new Pose2d(), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
