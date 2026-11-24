package frc.robot;

import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public final class Constants {
  public static final class ShooterConstants {
    public static final double FLYWHEEL_RPM = 3000;
    public static final int CURRENT_LIMIT_AMPS = 60;
    
    public static final double KP = 0.0003;
    public static final double KV = 0.00015;
    public static final PIDGains FLYWHEEL_PID = new PIDGains.Builder().kp(0.00055).build();
    public static final FeedforwardGains FLYWHEEL_FEEDFORWARD =
      new FeedforwardGains.Builder().kv(0.0019).build();

    public static final int MOTOR_ID = 20;
  }
}
