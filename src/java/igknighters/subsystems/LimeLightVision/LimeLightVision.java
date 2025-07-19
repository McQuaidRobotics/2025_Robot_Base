package igknighters.subsystems.LimeLightVision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class LimeLightVision {
  public abstract List<Integer> getSeenTags();
  public abstract String getName();
  public abstract Pose2d getPose();
}
