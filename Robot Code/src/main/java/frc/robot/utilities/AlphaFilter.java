package frc.robot.utilities;

/**
 * A filter to achieve smooth rate of change to a desired input speed through the equation: <br>
 * </br>
 * output = (alpha*input) + ((1-alpha)*oldspeed) <br>
 * </br>
 * Where alpha is a tunable coefficient.
 *
 * @author Adam N.
 */
public class AlphaFilter implements Filter {

  private double mAlphaForward, mAlphaBackward;
  private double mFilteredSpeedForward, mFilteredSpeedBackward;

  /**
   * @param The desired alpha coefficients
   */
  public AlphaFilter(double alphaFor, double alphaBack) {
    mAlphaForward = alphaFor;
    mAlphaBackward = alphaBack;
  }

  /**
   * @param The desired alpha coefficient
   */
  public AlphaFilter(double alpha) {
    mAlphaForward = alpha;
    mAlphaBackward = alpha;
  }

  /**
   * Calculates the correct output based on a desired value.
   *
   * @param input The desired value to reach
   * @return The calculated output based on previous inputs.
   */
  public double calculate(double input) {
    if(mAlphaForward != mAlphaBackward) {
      if(input >= 0) {
        mFilteredSpeedForward = (mAlphaForward * input) + ((1 - mAlphaForward) * mFilteredSpeedForward);
        return mFilteredSpeedForward;
      } else {
        mFilteredSpeedBackward = (mAlphaBackward * input) + ((1 - mAlphaBackward) * mFilteredSpeedBackward);
        return mFilteredSpeedBackward;
      }
    }
    mFilteredSpeedForward = (mAlphaForward * input) + ((1 - mAlphaForward) * mFilteredSpeedForward);
    return mFilteredSpeedForward;
  }

  /**
   * set the alpha gain forward.
   *
   * @param alpha The alpha gain forward
   */
  public void setAlphaForward(double alphaF) {
    mAlphaForward = alphaF;
  }

  /**
   * set the alpha gain backward.
   *
   * @param alpha The alpha gain backward
   */
  public void setAlphaBackward(double alphaB) {
    mAlphaBackward = alphaB;
  }

  @Override
  public void reset() {
    mFilteredSpeedForward = 0.0;
    mFilteredSpeedBackward = 0.0;
  }
}