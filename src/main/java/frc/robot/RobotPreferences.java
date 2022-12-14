package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

public class RobotPreferences {

  public static final SN_BooleanPreference displayPreferences = new SN_BooleanPreference("displayPreferences",
      false);

  public static final class prefDrivetrain {
    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 1);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.6);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6);

    public static final SN_DoublePreference thetaP = new SN_DoublePreference("thetaP", 10);
    public static final SN_DoublePreference thetaI = new SN_DoublePreference("thetaI", 0);
    public static final SN_DoublePreference thetaD = new SN_DoublePreference("thetaD", 0);
    public static final SN_DoublePreference thetaToleranceDegrees = new SN_DoublePreference("thetaToleranceDegrees", 0);

    public static final SN_DoublePreference transP = new SN_DoublePreference("transP", 1);
    public static final SN_DoublePreference transI = new SN_DoublePreference("transI", 0);
    public static final SN_DoublePreference transD = new SN_DoublePreference("transD", 0);

    public static final SN_DoublePreference transTolInches = new SN_DoublePreference("transTolInches", 2);

    public static final SN_DoublePreference transMaxFPS = new SN_DoublePreference("transMaxFPS", 5);
    public static final SN_DoublePreference transMaxFPSPS = new SN_DoublePreference("transMaxFPSPS", 2);

    // FPS: feet per second
    public static final SN_DoublePreference maxChassisSpeedFPS = new SN_DoublePreference(
        "maxChassisSpeedFPS", 16.3);
    // DPS: degrees per second
    public static final SN_DoublePreference maxRotationDPS = new SN_DoublePreference(
        "maxRotationDPS", 360);
    // DPSPS: DPS per second, or rotational acceleration (rename this at some point)
    public static final SN_DoublePreference maxRotationDPSPS = new SN_DoublePreference(
        "maxRotationDPSPS", 180);

    public static final SN_DoublePreference percentOfMaxSpeedToSteer = new SN_DoublePreference(
        "percentOfMaxSpeedToSteer", 0.01);

    // applied on joysticks, so units are from -1 to 1
    public static final SN_DoublePreference driveRateLimit = new SN_DoublePreference("driveRateLimit", 100);
    public static final SN_DoublePreference steerRateLimit = new SN_DoublePreference("steerRateLimit", 100);

    public static final SN_DoublePreference controllerDeadband = new SN_DoublePreference(
        "controllerDeadband", 0.01);
    public static final SN_DoublePreference absSteerControllerDeadband = new SN_DoublePreference(
        "absSteerControllerDeadband", 0.5);

    public static final SN_DoublePreference absSteerOffsetDegrees = new SN_DoublePreference(
        "absSteerOffsetDegrees", 90);

    public static final SN_DoublePreference stateStdDevsMeters = new SN_DoublePreference(
        "stateStdDevsMeters", 0.05);
    public static final SN_DoublePreference stateStdDevsDegrees = new SN_DoublePreference(
        "stateStdDevsThetaDegrees", 5);
    public static final SN_DoublePreference localMeasurementStdDevsDegrees = new SN_DoublePreference(
        "localMeasurementStdDevsDegrees", 0.01);
    public static final SN_DoublePreference visionMeasurementStdDevsMeters = new SN_DoublePreference(
        "visionMeasurementStdDevsMeters", 0.5);
    public static final SN_DoublePreference visionMeasurementStdDevsDegrees = new SN_DoublePreference(
        "visionMeasurementStdDevsDegrees", 30);

  }

  public static final class prefVision {
    // Camera pose relative to robot
    public static final SN_DoublePreference cameraXPosition = new SN_DoublePreference("cameraXPosition", 0);
    public static final SN_DoublePreference cameraYPosition = new SN_DoublePreference(
        "cameraYPosition", Units.inchesToMeters(11));
    public static final SN_DoublePreference cameraZPosition = new SN_DoublePreference(
        "cameraZPosition", Units.inchesToMeters(8.5));
    public static final SN_DoublePreference cameraPitch = new SN_DoublePreference("cameraPitch", 0);
    public static final SN_DoublePreference cameraYaw = new SN_DoublePreference("cameraYaw", 0);
    public static final SN_DoublePreference cameraRoll = new SN_DoublePreference("cameraRoll", 0);
  }
}