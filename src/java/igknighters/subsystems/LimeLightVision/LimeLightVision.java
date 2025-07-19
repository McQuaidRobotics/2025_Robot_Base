package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

public abstract class LimeLightVision {
  public abstract List<Integer> getSeenTags();

  public abstract String getName();

  public abstract Pose2d getPose();
}
