package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

public class RobotPreferences {

  public static final SN_BooleanPreference displayPreferences = new SN_BooleanPreference(
      "displayPreferences", false);

  public static final class prefDrivetrain {
    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0.0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 1.0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.6);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6);

    public static final SN_DoublePreference autoThetaP = new SN_DoublePreference("autoThetaP", 1);
    public static final SN_DoublePreference autoThetaI = new SN_DoublePreference("autoThetaI", 0);
    public static final SN_DoublePreference autoThetaD = new SN_DoublePreference("autoThetaD", 0);

    public static final SN_DoublePreference autoTransP = new SN_DoublePreference("autoTransP", 18);
    public static final SN_DoublePreference autoTransI = new SN_DoublePreference("autoTransI", 0);
    public static final SN_DoublePreference autoTransD = new SN_DoublePreference("autoTransD", 0);
    // maximum translational speed in feet per second
    public static final SN_DoublePreference autoMaxSpeedFeet = new SN_DoublePreference(
        "autoMaxSpeedFeet", 5);
    // maximum translationl acceleration in feet per second per second
    public static final SN_DoublePreference autoMaxAccelFeet = new SN_DoublePreference(
        "autoMaxAccelFeet", 2);

    public static final SN_DoublePreference teleThetaP = new SN_DoublePreference("teleThetaP", 10);
    public static final SN_DoublePreference teleThetaI = new SN_DoublePreference("teleThetaI", 0);
    public static final SN_DoublePreference teleThetaD = new SN_DoublePreference("teleThetaD", 0);
    public static final SN_DoublePreference teleThetaToleranceDegrees = new SN_DoublePreference(
        "teleThetaToleranceDegrees", 0);
    public static final SN_DoublePreference teleThetaMaxRotSpeedDegrees = new SN_DoublePreference(
        "teleThetaMaxRotSpeedDegrees", 360);
    public static final SN_DoublePreference teleThetaMaxRotAccelDegrees = new SN_DoublePreference(
        "teleThetaMaxRotAccelDegrees", 360);

    public static final SN_DoublePreference teleMaxSpeedFeet = new SN_DoublePreference(
        "teleMaxSpeedFeet", 13.5);
    public static final SN_DoublePreference teleMaxRotSpeedDegrees = new SN_DoublePreference(
        "teleMaxRotSpeedDegrees", 360);

    public static final SN_DoublePreference percentOfMaxSpeedToSteer = new SN_DoublePreference(
        "percentOfMaxSpeedToSteer", 0.01);

    public static final SN_DoublePreference slewMaxAccelFeet = new SN_DoublePreference(
        "slewMaxAccelFeet", 100);
    public static final SN_DoublePreference slewMaxRotAccelDegrees = new SN_DoublePreference(
        "slewMaxRotAccelDegrees", 3600);

    // deadband for my (ian) personal xbox controller. need to retune for each
    // controller
    public static final SN_DoublePreference rightStickDeadband = new SN_DoublePreference(
        "rightStickDeadband", 0.1);
    public static final SN_DoublePreference leftStickDeadband = new SN_DoublePreference(
        "leftStickDeadband", 0.13);
    public static final SN_DoublePreference rightTriggerDeadband = new SN_DoublePreference(
        "rightTriggerDeadband", 0.01);
    public static final SN_DoublePreference leftTriggerDeadband = new SN_DoublePreference(
        "leftTriggerDeadband", 0.01);
    public static final SN_DoublePreference absSteerControllerDeadband = new SN_DoublePreference(
        "absSteerControllerDeadband", 0.5);

    public static final SN_DoublePreference stateStdDevsMeters = new SN_DoublePreference(
        "stateStdDevsMeters", 0.05);
    public static final SN_DoublePreference stateStdDevsDegrees = new SN_DoublePreference(
        "stateStdDevsDegrees", 5);
    public static final SN_DoublePreference localMeasurementStdDevsDegrees = new SN_DoublePreference(
        "localMeasurementStdDevsDegrees", 0.01);
    public static final SN_DoublePreference visionMeasurementStdDevsMeters = new SN_DoublePreference(
        "visionMeasurementStdDevsMeters", 0.5);
    public static final SN_DoublePreference visionMeasurementStdDevsDegrees = new SN_DoublePreference(
        "visionMeasurementStdDevsDegrees", 30);

    // Is the drive motor velocity controlled using open or closed loop control
    public static final SN_BooleanPreference isDriveOpenLoop = new SN_BooleanPreference("isDriveOpenLoop", true);
  }

  public static final class prefVision {

    public static final SN_BooleanPreference useVision = new SN_BooleanPreference(
        "useVision", false);

    // Camera pose relative to robot
    public static final SN_DoublePreference cameraXPosition = new SN_DoublePreference(
        "cameraXPosition", 0);
    public static final SN_DoublePreference cameraYPosition = new SN_DoublePreference(
        "cameraYPosition", Units.inchesToMeters(11));
    public static final SN_DoublePreference cameraZPosition = new SN_DoublePreference(
        "cameraZPosition", Units.inchesToMeters(8.5));
    public static final SN_DoublePreference cameraPitch = new SN_DoublePreference("cameraPitch", 0);
    public static final SN_DoublePreference cameraYaw = new SN_DoublePreference("cameraYaw", 0);
    public static final SN_DoublePreference cameraRoll = new SN_DoublePreference("cameraRoll", 0);

    public static final SN_DoublePreference chasingTagID = new SN_DoublePreference("chasingTagID", 1);
    public static final SN_DoublePreference goalDistToTag = new SN_DoublePreference("distanceInFrontOfTag", 2);
  }
}