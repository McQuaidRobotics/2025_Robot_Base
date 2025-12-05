package igknighters.util.geom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

/** A trigger thats condition is based on the position of the robot */
public class PolyTrigger extends Trigger {
  private final Polygon2d polygon;

  public PolyTrigger(Supplier<Pose2d> poseSupplier, Polygon2d polygon) {
    super(() -> polygon.contains(poseSupplier.get().getTranslation()));
    this.polygon = polygon;
  }

  public PolyTrigger(Supplier<Pose2d> poseSupplier, Translation2d... polygonVertecies) {
    this(poseSupplier, new Polygon2d(polygonVertecies));
  }

  public Polygon2d getPolygon() {
    return polygon;
  }
}
