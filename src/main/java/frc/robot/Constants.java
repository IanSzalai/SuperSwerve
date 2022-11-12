package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap.mapDrivetrain;

public final class Constants {

  public static final class constDrivetrain {
    // ORDER OF MODULES:
    // FRONT LEFT
    // FRONT RIGHT
    // BACK LEFT
    // BACK RIGHT

    public static final double TRACK_WIDTH = Units.inchesToMeters(19.75); // side to side
    public static final double WHEEL_BASE = Units.inchesToMeters(15.75); // front to back

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.75; // SDS MK4 L2
    public static final double STEER_GEAR_RATIO = 12.8; // SDS MK4 & MK3 (all drive gearings)

    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
    public static final NeutralMode STEER_NEUTRAL_MODE = NeutralMode.Coast;

    public static final boolean DRIVE_INVERT = false;
    public static final boolean STEER_INVERT = false;
    public static final boolean STEER_ENCODER_INVERT = false;
    public static final boolean GYRO_INVERT = false;

    public static final class Module0 {
      private static final double STEER_ENCODER_OFFSET = Units.degreesToRadians(37.35);
      private static final Translation2d POSITION = new Translation2d(-TRACK_WIDTH / 2, WHEEL_BASE / 2);
      public static final SN_SwerveModuleConstants CONSTANTS = new SN_SwerveModuleConstants(
          mapDrivetrain.FRONT_LEFT_DRIVE_CAN,
          mapDrivetrain.FRONT_LEFT_STEER_CAN,
          mapDrivetrain.FRONT_LEFT_STEER_ENCODER_DIO,
          STEER_ENCODER_OFFSET,
          POSITION,
          0);
    }

    public static final class Module1 {
      private static final double STEER_ENCODER_OFFSET = Units.degreesToRadians(10.45);
      private static final Translation2d POSITION = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2);
      public static final SN_SwerveModuleConstants CONSTANTS = new SN_SwerveModuleConstants(
          mapDrivetrain.FRONT_RIGHT_DRIVE_CAN,
          mapDrivetrain.FRONT_RIGHT_STEER_CAN,
          mapDrivetrain.FRONT_RIGHT_STEER_ENCODER_DIO,
          STEER_ENCODER_OFFSET,
          POSITION,
          1);
    }

    public static final class Module2 {
      private static final double STEER_ENCODER_OFFSET = Units.degreesToRadians(38.75);
      private static final Translation2d POSITION = new Translation2d(-TRACK_WIDTH / 2, -WHEEL_BASE / 2);
      public static final SN_SwerveModuleConstants CONSTANTS = new SN_SwerveModuleConstants(
          mapDrivetrain.BACK_LEFT_DRIVE_CAN,
          mapDrivetrain.BACK_LEFT_STEER_CAN,
          mapDrivetrain.BACK_LEFT_STEER_ENCODER_DIO,
          STEER_ENCODER_OFFSET,
          POSITION,
          2);
    }

    public static final class Module3 {
      private static final double STEER_ENCODER_OFFSET = Units.degreesToRadians(58.88);
      private static final Translation2d POSITION = new Translation2d(TRACK_WIDTH / 2, -WHEEL_BASE / 2);
      public static final SN_SwerveModuleConstants CONSTANTS = new SN_SwerveModuleConstants(
          mapDrivetrain.BACK_RIGHT_DRIVE_CAN,
          mapDrivetrain.BACK_RIGHT_STEER_CAN,
          mapDrivetrain.BACK_RIGHT_STEER_ENCODER_DIO,
          STEER_ENCODER_OFFSET,
          POSITION,
          3);

    }

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        Module0.POSITION,
        Module1.POSITION,
        Module2.POSITION,
        Module3.POSITION);
  }
}
