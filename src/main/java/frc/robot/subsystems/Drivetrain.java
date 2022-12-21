package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPreferences;
import frc.robot.SN_SwerveModule;
import frc.robot.Constants.Module0;
import frc.robot.Constants.Module1;
import frc.robot.Constants.Module2;
import frc.robot.Constants.Module3;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

public class Drivetrain extends SubsystemBase {

  private SN_SwerveModule[] swerveModules;
  private PigeonIMU pigeon;
  private SwerveDrivePoseEstimator poseEstimator;

  public SlewRateLimiter driveXSlewRateLimiter;
  public SlewRateLimiter driveYSlewRateLimiter;
  public SlewRateLimiter steerSlewRateLimiter;

  public ProfiledPIDController thetaPIDController;
  public ProfiledPIDController xTransPIDController;
  public ProfiledPIDController yTransPIDController;

  public Field2d field;

  private boolean fieldRelative;

  public Drivetrain() {

    swerveModules = new SN_SwerveModule[] {
        new SN_SwerveModule(Module0.CONSTANTS),
        new SN_SwerveModule(Module1.CONSTANTS),
        new SN_SwerveModule(Module2.CONSTANTS),
        new SN_SwerveModule(Module3.CONSTANTS)
    };

    pigeon = new PigeonIMU(mapDrivetrain.PIGEON_CAN);
    zeroGyroYaw();

    poseEstimator = new SwerveDrivePoseEstimator(
        getGyroYaw(),
        new Pose2d(),
        Constants.SWERVE_KINEMATICS,
        VecBuilder.fill(
            prefDrivetrain.stateStdDevsMeters.getValue(),
            prefDrivetrain.stateStdDevsMeters.getValue(),
            Units.degreesToRadians(prefDrivetrain.stateStdDevsDegrees.getValue())),
        VecBuilder.fill(
            Units.degreesToRadians(prefDrivetrain.localMeasurementStdDevsDegrees.getValue())),
        VecBuilder.fill(
            prefDrivetrain.visionMeasurementStdDevsMeters.getValue(),
            prefDrivetrain.visionMeasurementStdDevsMeters.getValue(),
            Units.degreesToRadians(prefDrivetrain.visionMeasurementStdDevsDegrees.getValue())));

    driveXSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.driveSecondsToMax.getValue());
    driveYSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.driveSecondsToMax.getValue());
    steerSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.steerSecondsToMax.getValue());

    xTransPIDController = new ProfiledPIDController(
        prefDrivetrain.transP.getValue(),
        prefDrivetrain.transI.getValue(),
        prefDrivetrain.transD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.transMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.transMaxAccelFeet.getValue())));

    yTransPIDController = new ProfiledPIDController(
        prefDrivetrain.transP.getValue(),
        prefDrivetrain.transI.getValue(),
        prefDrivetrain.transD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.transMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.transMaxAccelFeet.getValue())));

    thetaPIDController = new ProfiledPIDController(
        prefDrivetrain.thetaP.getValue(),
        prefDrivetrain.thetaI.getValue(),
        prefDrivetrain.thetaD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue()),
            Units.degreesToRadians(prefDrivetrain.maxChassisRotAccelDegrees.getValue())));

    field = new Field2d();

    fieldRelative = true;

    configure();
  }

  /**
   * Configure each swerve module and the Pigeon IMU
   */
  public void configure() {
    for (SN_SwerveModule mod : swerveModules) {
      mod.configure();
    }
    pigeon.configFactoryDefault();
    resetSteerMotorEncodersToAbsolute();

    poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(
            prefDrivetrain.visionMeasurementStdDevsMeters.getValue(),
            prefDrivetrain.visionMeasurementStdDevsMeters.getValue(),
            Units.degreesToRadians(prefDrivetrain.visionMeasurementStdDevsDegrees.getValue())));

    driveXSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.driveSecondsToMax.getValue());
    driveYSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.driveSecondsToMax.getValue());
    steerSlewRateLimiter = new SlewRateLimiter(1 / prefDrivetrain.steerSecondsToMax.getValue());

    xTransPIDController.setPID(
        prefDrivetrain.transP.getValue(),
        prefDrivetrain.transI.getValue(),
        prefDrivetrain.transD.getValue());

    xTransPIDController.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.transMaxSpeedFeet.getValue()),
        Units.feetToMeters(prefDrivetrain.transMaxAccelFeet.getValue())));

    xTransPIDController.setTolerance(Units.inchesToMeters(prefDrivetrain.transToleranceInches.getValue()));

    yTransPIDController.setPID(
        prefDrivetrain.transP.getValue(),
        prefDrivetrain.transI.getValue(),
        prefDrivetrain.transD.getValue());

    yTransPIDController.setConstraints(new TrapezoidProfile.Constraints(
        Units.feetToMeters(prefDrivetrain.transMaxSpeedFeet.getValue()),
        Units.feetToMeters(prefDrivetrain.transMaxAccelFeet.getValue())));

    yTransPIDController.setTolerance(Units.inchesToMeters(prefDrivetrain.transToleranceInches.getValue()));

    thetaPIDController.setPID(
        prefDrivetrain.thetaP.getValue(),
        prefDrivetrain.thetaI.getValue(),
        prefDrivetrain.thetaD.getValue());

    thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    thetaPIDController.setTolerance(Units.degreesToRadians(prefDrivetrain.thetaToleranceDegrees.getValue()));

    thetaPIDController.setConstraints(new TrapezoidProfile.Constraints(
        Units.degreesToRadians(prefDrivetrain.maxChassisRotSpeedDegrees.getValue()),
        Units.degreesToRadians(prefDrivetrain.maxChassisRotAccelDegrees.getValue())));

    thetaPIDController.reset(getPose().getRotation().getRadians());
  }

  public void driveAlignAngle(Pose2d velocity) {
    thetaPIDController.setGoal(new TrapezoidProfile.State(velocity.getRotation().getRadians(), 0));
    double goalAngle = thetaPIDController.calculate(getPose().getRotation().getRadians());
    Pose2d newVelocity = new Pose2d(velocity.getTranslation(), new Rotation2d(goalAngle));
    drive(newVelocity);
  }

  /**
   * Drive the drivetrain
   * 
   * @param velocity Desired translational and rotational velocity in
   *                 meters and radians per second respectively
   */
  public void drive(Pose2d velocity) {

    Pose2d slewedVelocity = new Pose2d(
        driveXSlewRateLimiter.calculate(velocity.getX()),
        driveYSlewRateLimiter.calculate(velocity.getY()),
        new Rotation2d(steerSlewRateLimiter.calculate(velocity.getRotation().getRadians())));

    SmartDashboard.putNumber(".slewed x", slewedVelocity.getX());
    SmartDashboard.putNumber(".slewed y", slewedVelocity.getY());
    SmartDashboard.putNumber(".slewed degrees", slewedVelocity.getRotation().getDegrees());

    ChassisSpeeds chassisSpeeds;

    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          slewedVelocity.getX(),
          slewedVelocity.getY(),
          slewedVelocity.getRotation().getRadians(),
          getGyroYaw());
    } else {
      chassisSpeeds = new ChassisSpeeds(
          slewedVelocity.getX(),
          slewedVelocity.getY(),
          slewedVelocity.getRotation().getRadians());
    }
    SwerveModuleState[] states = Constants.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    // mutates states with desaturated wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(states, constDrivetrain.MAX_SPEED);

    for (SN_SwerveModule mod : swerveModules) {
      mod.setDesiredState(states[mod.moduleNumber], prefDrivetrain.isDriveOpenLoop.getValue());
    }
  }

  /**
   * Directly set the state of each swerve module. Uses closed loop control for
   * drive velocity
   * 
   * @param desiredStates List of each desired state
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constDrivetrain.MAX_SPEED);

    for (SN_SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetSteerMotorEncodersToAbsolute() {
    for (SN_SwerveModule mod : swerveModules) {
      mod.resetSteerMotorEncoderToAbsolute();
    }
  }

  /**
   * Get the state (velocity and angle) of each swerve module
   * 
   * @return State of each swerve module
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState states[] = new SwerveModuleState[swerveModules.length];

    for (SN_SwerveModule mod : swerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  /**
   * Set the desired translational velocity to be field relative
   */
  public void setFieldRelative() {
    fieldRelative = true;
  }

  /**
   * Set the desired translational velocity to be robot relative
   */
  public void setRobotRelative() {
    fieldRelative = false;
  }

  public void toggleFieldRelative() {
    fieldRelative = fieldRelative ? false : true;
  }

  /**
   * Get the yaw as reported by the Pigeon IMU
   * 
   * @return Yaw of drivetrain
   */
  public Rotation2d getGyroYaw() {
    double yawDegress = pigeon.getYaw();
    yawDegress *= constDrivetrain.GYRO_INVERT ? -1 : 1;
    return Rotation2d.fromDegrees(yawDegress);
  }

  public void zeroGyroYaw() {
    pigeon.setYaw(0);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(pose, getGyroYaw());
    for (SN_SwerveModule mod : swerveModules) {
      mod.resetDriveEncoderCount();
    }
  }

  /**
   * Updates the pose estimator with the current robot uptime, the gyro yaw, and
   * each swerve module state.
   * <p>
   * This method should be called every loop.
   */
  public void updatePoseEstimator() {
    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        getGyroYaw(),
        getModuleStates());
  }

  /**
   * Add a vision measurement to the pose estimator. This will not directly set
   * the pose, it will simply be another data point for the pose estimator to use.
   * 
   * @param visionRobotPose Pose of robot as calculated by the vision system
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(
        visionRobotPose,
        timestampSeconds);
  }

  @Override
  public void periodic() {

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
    if (RobotPreferences.displayPreferences.getValue()) {

      for (SN_SwerveModule mod : swerveModules) {

        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Encoder",
            mod.getSteerEncoder().getDegrees());
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Drive Motor Velocity ",
            Units.metersToFeet(mod.getState().speedMetersPerSecond));
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Motor Angle ",
            mod.getState().angle.getDegrees());

      }

      SmartDashboard.putNumber("Drivetrain Gyro Yaw", getGyroYaw().getDegrees());

      SmartDashboard.putNumber("Drivetrain Pose X", getPose().getX());
      SmartDashboard.putNumber("Drivetrain Pose Y", getPose().getY());
      SmartDashboard.putNumber("Drivetrain Pose Rotation", getPose().getRotation().getDegrees());

      SmartDashboard.putBoolean("Drivetrain field relative", fieldRelative);

    }
  }
}
