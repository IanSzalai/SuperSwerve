package frc.robot;

public class RobotMap {

  public static final class mapControllers {
    public static final int DRIVER = 0;
  }

  public static final class mapDrivetrain {
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_STEER_ENCODER_DIO = 0;

    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_STEER_ENCODER_DIO = 1;

    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_STEER_ENCODER_DIO = 2;

    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_STEER_ENCODER_DIO = 3;

    public static final int PIGEON_CAN = 8;
  }
}
