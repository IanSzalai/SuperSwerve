package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.utils.CTREModuleState;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

/**
 * Coaxial swerve module with TalonFX's for both drive and steering.
 * Uses DutyCycleEncoder for the absolute steering encoder (CTRE SRX Mag
 * Encoder).
 */
public class SN_SwerveModule {

  public int moduleNumber;
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private DutyCycleEncoder steerEncoder;
  private double steerEncoderOffset;

  private TalonFXConfiguration driveConfiguration;
  private TalonFXConfiguration steerConfiguration;

  private Rotation2d lastAngle;

  public SN_SwerveModule(SN_SwerveModuleConstants moduleConstants) {
    moduleNumber = moduleConstants.moduleNumber;

    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    steerMotor = new TalonFX(moduleConstants.steerMotorID);
    steerEncoder = new DutyCycleEncoder(moduleConstants.steerEncoderID);
    steerEncoderOffset = moduleConstants.steerEncoderOffset;

    lastAngle = getState().angle;

    configure();
  }

  /**
   * Configure both motors and encoder of module
   */
  public void configure() {
    driveMotor.configFactoryDefault();
    driveConfiguration.slot0.kF = prefDrivetrain.driveF.getValue();
    driveConfiguration.slot0.kP = prefDrivetrain.driveP.getValue();
    driveConfiguration.slot0.kI = prefDrivetrain.driveI.getValue();
    driveConfiguration.slot0.kD = prefDrivetrain.driveD.getValue();
    driveMotor.configAllSettings(driveConfiguration);
    driveMotor.setNeutralMode(constDrivetrain.DRIVE_NEUTRAL_MODE);
    driveMotor.setInverted(constDrivetrain.DRIVE_INVERT);

    steerMotor.configFactoryDefault();
    steerConfiguration.slot0.kP = prefDrivetrain.steerP.getValue();
    steerConfiguration.slot0.kI = prefDrivetrain.steerI.getValue();
    steerConfiguration.slot0.kD = prefDrivetrain.steerD.getValue();
    steerMotor.configAllSettings(steerConfiguration);
    steerMotor.setNeutralMode(constDrivetrain.STEER_NEUTRAL_MODE);
    steerMotor.setInverted(constDrivetrain.STEER_INVERT);
  }

  /**
   * Set the desired state of the swerve module. The state includes the velocity
   * and angle of the module. The state that is actually given to the motors will
   * be optimized from the desired state.
   * 
   * @param desiredState Desired state of the module
   * @param isOpenLoop   Is the drive motor velocity controlled using
   *                     open or closed loop control
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    SwerveModuleState state = CTREModuleState.optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      double percentOutput = state.speedMetersPerSecond / Units.feetToMeters(prefDrivetrain.maxSpeedFPS.getValue());
      driveMotor.set(ControlMode.PercentOutput, percentOutput);

    } else {
      double velocity = SN_Math.MPSToFalcon(
          state.speedMetersPerSecond,
          constDrivetrain.WHEEL_CIRCUMFERENCE,
          constDrivetrain.DRIVE_GEAR_RATIO);
      driveMotor.set(ControlMode.Velocity, velocity);
    }

    double angle = SN_Math.degreesToFalcon(state.angle.getDegrees(), constDrivetrain.STEER_GEAR_RATIO);

    if (Math.abs(state.speedMetersPerSecond) <= prefDrivetrain.percentOfMaxSpeedToSteer.getValue()) {
      angle = SN_Math.degreesToFalcon(lastAngle.getDegrees(), constDrivetrain.STEER_GEAR_RATIO);
    }

    steerMotor.set(ControlMode.Position, angle);

  }

  /**
   * Reset the steer motor encoder to match the angle of the absolute steer
   * encoder
   */
  public void resetSteerMotorEncoderToAbsolute() {
    double absoluteEncoderCount = SN_Math.degreesToFalcon(
        getSteerEncoder().getDegrees(),
        constDrivetrain.STEER_GEAR_RATIO);

    steerMotor.setSelectedSensorPosition(absoluteEncoderCount);
  }

  /**
   * Get the absolute steer encoder angle
   * 
   * @return steer encoder angle as Rotation2d
   */
  public Rotation2d getSteerEncoder() {

    double rotations = steerEncoder.getAbsolutePosition();
    double radians = Units.rotationsToRadians(rotations);
    double correctedRadians = radians - steerEncoderOffset;

    correctedRadians *= constDrivetrain.STEER_ENCODER_INVERT ? -1 : 1;

    return new Rotation2d(correctedRadians);
  }

  /**
   * Get the curret state of the swerve module. State includes velocity and angle
   * 
   * @return State of swerve module
   */
  public SwerveModuleState getState() {
    double velocity = SN_Math.falconToMPS(
        driveMotor.getSelectedSensorVelocity(),
        constDrivetrain.WHEEL_CIRCUMFERENCE,
        constDrivetrain.DRIVE_GEAR_RATIO);

    Rotation2d angle = Rotation2d.fromDegrees(
        SN_Math.falconToDegrees(
            steerMotor.getSelectedSensorPosition(),
            constDrivetrain.STEER_GEAR_RATIO));

    return new SwerveModuleState(velocity, angle);
  }

}
