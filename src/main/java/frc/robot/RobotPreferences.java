package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

public class RobotPreferences {

  public static final class prefDrivetrain {
    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 1);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0);

    public static final SN_DoublePreference steerP = new SN_DoublePreference("steerP", 0.6);
    public static final SN_DoublePreference steerI = new SN_DoublePreference("steerI", 0);
    public static final SN_DoublePreference steerD = new SN_DoublePreference("steerD", 6);

    // FPS: feet per second
    public static final SN_DoublePreference maxSpeedFPS = new SN_DoublePreference(
        "maxSpeedMPS", 16.3);
    // DPS: degrees per second
    public static final SN_DoublePreference maxRotationDPS = new SN_DoublePreference(
        "maxRotationDPS", 360);

    public static final SN_DoublePreference percentOfMaxSpeedToSteer = new SN_DoublePreference(
        "percentOfMaxSpeedToSteer", 0.01);

    // applied on joysticks, so units are from -1 to 1
    public static final SN_DoublePreference driveRateLimit = new SN_DoublePreference("driveRateLimit", 0);
    public static final SN_DoublePreference steerRateLimit = new SN_DoublePreference("steerRateLimit", 0);
  }

}