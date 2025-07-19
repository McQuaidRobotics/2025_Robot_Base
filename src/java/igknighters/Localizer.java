package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import wayfinder.poseEst.TwistyPoseEst.VisionScalars;

public class Localizer {
  private static final VisionScalars VISION_SCALARS = new VisionScalars(0.9, 0.05, 1.0);

  private final Pose2d robotPose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0));

  public static boolean withinTolerance(Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
    if (Math.abs(toleranceRadians) > Math.PI) {
      return true;
    }
    double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
    // cos(θ) >= cos(tolerance) means |θ| <= tolerance, for tolerance in [-pi, pi], as pre-checked
    // above.
    return dot > Math.cos(toleranceRadians);
  }

  public Localizer(){

  }
}
