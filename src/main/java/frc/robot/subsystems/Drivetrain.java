package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.frcteam3255.utils.SN_Math;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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

  private ProfiledPIDController xTransPIDController;
  private ProfiledPIDController yTransPIDController;
  private ProfiledPIDController thetaPIDController;

  public Field2d field;

  public SwerveAutoBuilder autoBuilder;
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

    driveXSlewRateLimiter = new SlewRateLimiter(Units.feetToMeters(prefDrivetrain.slewMaxAccelFeet.getValue()));
    driveYSlewRateLimiter = new SlewRateLimiter(Units.feetToMeters(prefDrivetrain.slewMaxAccelFeet.getValue()));
    steerSlewRateLimiter = new SlewRateLimiter(
        Units.degreesToRadians(prefDrivetrain.slewMaxRotAccelDegrees.getValue()));

    xTransPIDController = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccelFeet.getValue())));
    yTransPIDController = new ProfiledPIDController(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccelFeet.getValue())));
    thetaPIDController = new ProfiledPIDController(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue(),
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(prefDrivetrain.teleThetaMaxRotSpeedDegrees.getValue()),
            Units.degreesToRadians(prefDrivetrain.teleThetaMaxRotAccelDegrees.getValue())));

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

    driveXSlewRateLimiter = new SlewRateLimiter(Units.feetToMeters(prefDrivetrain.slewMaxAccelFeet.getValue()));
    driveYSlewRateLimiter = new SlewRateLimiter(Units.feetToMeters(prefDrivetrain.slewMaxAccelFeet.getValue()));
    steerSlewRateLimiter = new SlewRateLimiter(
        Units.degreesToRadians(prefDrivetrain.slewMaxRotAccelDegrees.getValue()));

    xTransPIDController.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());

    xTransPIDController.setTolerance(Units.inchesToMeters(prefDrivetrain.teleTransToleranceInches.getValue()));

    xTransPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccelFeet.getValue())));
    xTransPIDController.reset(getPose().getX());

    yTransPIDController.setPID(
        prefDrivetrain.teleTransP.getValue(),
        prefDrivetrain.teleTransI.getValue(),
        prefDrivetrain.teleTransD.getValue());

    yTransPIDController.setTolerance(Units.inchesToMeters(prefDrivetrain.teleTransToleranceInches.getValue()));

    yTransPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            Units.feetToMeters(prefDrivetrain.teleTransMaxSpeedFeet.getValue()),
            Units.feetToMeters(prefDrivetrain.teleTransMaxAccelFeet.getValue())));
    yTransPIDController.reset(getPose().getY());

    thetaPIDController.setPID(
        prefDrivetrain.teleThetaP.getValue(),
        prefDrivetrain.teleThetaI.getValue(),
        prefDrivetrain.teleThetaD.getValue());

    thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    thetaPIDController.setTolerance(Units.degreesToRadians(prefDrivetrain.teleThetaToleranceDegrees.getValue()));

    thetaPIDController.setConstraints(new TrapezoidProfile.Constraints(
        Units.degreesToRadians(prefDrivetrain.teleThetaMaxRotSpeedDegrees.getValue()),
        Units.degreesToRadians(prefDrivetrain.teleThetaMaxRotAccelDegrees.getValue())));

    thetaPIDController.reset(getPose().getRotation().getRadians());

    autoBuilder = new SwerveAutoBuilder(
        this::getPose,
        this::resetPose,
        Constants.SWERVE_KINEMATICS,
        new PIDConstants(
            prefDrivetrain.autoTransP.getValue(),
            prefDrivetrain.autoTransI.getValue(),
            prefDrivetrain.autoTransD.getValue()),
        new PIDConstants(
            prefDrivetrain.autoThetaP.getValue(),
            prefDrivetrain.autoThetaI.getValue(),
            prefDrivetrain.autoThetaD.getValue()),
        this::setModuleStates,
        new HashMap<>(),
        this);
  }

  public void driveAlignAngle(Pose2d velocity) {
    thetaPIDController.setGoal(new TrapezoidProfile.State(velocity.getRotation().getRadians(), 0.0));
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
          getPose().getRotation());
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

  public void driveToPosition(Pose2d position) {
    xTransPIDController.setGoal(new TrapezoidProfile.State(position.getX(), 0.0));
    yTransPIDController.setGoal(new TrapezoidProfile.State(position.getY(), 0.0));

    Pose2d velocity = new Pose2d(
        xTransPIDController.calculate(getPose().getX()),
        yTransPIDController.calculate(getPose().getY()),
        position.getRotation());

    driveAlignAngle(velocity);
  }

  public void resetPID() {
    xTransPIDController.reset(new TrapezoidProfile.State(getPose().getX(), 0.0));
    yTransPIDController.reset(new TrapezoidProfile.State(getPose().getY(), 0.0));
    thetaPIDController.reset(new TrapezoidProfile.State(getPose().getRotation().getRadians(), 0.0));
  }

  public void neutralOutputs() {
    for (SN_SwerveModule mod : swerveModules) {
      mod.neutralDriveOutput();
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

        // steer encoder
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Encoder",
            mod.getSteerEncoder().getDegrees());
        // drive motor velocity
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Drive Motor Velocity ",
            Units.metersToFeet(Math.abs(mod.getState().speedMetersPerSecond)));
        // steer motor angle
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Steer Motor Angle ",
            mod.getState().angle.getDegrees());
        // drive motor closed loop error
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Drive Motor Error ",
            Units.metersToFeet(SN_Math.falconToMPS(Math.abs(mod.getDriveMotorClosedLoopError()),
                constDrivetrain.WHEEL_CIRCUMFERENCE, constDrivetrain.DRIVE_GEAR_RATIO)));
        // drive motor closed loop goal velocity
        SmartDashboard.putNumber("Drivetrain Module " + mod.moduleNumber + " Drive Motor Goal Velocity",
            Math.abs(Units.metersToFeet(mod.goalVelocity)));

      }

      SmartDashboard.putNumber("Drivetrain Gyro Yaw", getGyroYaw().getDegrees());

      SmartDashboard.putNumber("Drivetrain Pose X", getPose().getX());
      SmartDashboard.putNumber("Drivetrain Pose Y", getPose().getY());
      SmartDashboard.putNumber("Drivetrain Pose Rotation", getPose().getRotation().getDegrees());

      SmartDashboard.putBoolean("Drivetrain field relative", fieldRelative);

      SmartDashboard.putNumber("Drivetrain xTransPID", xTransPIDController.getP());

    }
  }
}
