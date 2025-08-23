package frc.robot.util;

/**
 * General utility methods for the robot project.
 */
public final class Utils {

  private Utils() {
    // Prevent instantiation
  }

  /**
   * Applies a deadband filter to joystick values to ignore small inputs.
   *
   * @param value the joystick input value
   * @param cutoff the deadband cutoff threshold (0.0–1.0)
   * @return 0.0 if within the deadband, otherwise a scaled value
   */
  public static double deadbandReturn(double JoystickValue, double DeadbandCutOff) {
    double deadbandReturn;
      if (JoystickValue<DeadbandCutOff&&JoystickValue>(DeadbandCutOff*(-1))) {
      deadbandReturn=0; // if less than the deadband cutoff, return 0, if greater than the negative deadband cutoff, return 0
    }
    else {
      deadbandReturn=(JoystickValue- // initially in one of two ranges: [DeadbandCutOff,1] or -1,-DeadBandCutOff]
      (Math.abs(JoystickValue)/JoystickValue // 1 if JoystickValue > 0, -1 if JoystickValue < 0 (abs(x)/x); could use Math.signum(JoystickValue) instead
       *DeadbandCutOff // multiply by the sign so that for >0, it comes out to - (DeadBandCutOff), and for <0 it comes to - (-DeadBandCutOff)
      )
     ) // now in either [0,1-DeadBandCutOff] or -1+DeadBandCutOff,0]
     /(1-DeadbandCutOff); // scale to [0,1] or -1,0]
    }
      return deadbandReturn;
    }
}
