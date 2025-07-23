package igknighters.subsystems.LimeLightVision;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.LimelightHelpers;
import igknighters.subsystems.Subsystems;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVision implements Subsystems.SharedSubsystem {
  private final List<String> cameraNames;
  private double lastTimeStamp = 0.0;

  public LimeLightVision(String... cameraNames) {
    this.cameraNames = new ArrayList<>();
    for (String cameraName : cameraNames) {
      this.cameraNames.add(cameraName);
    }
  }

  public Pose2d getRobotPoseFromVision(double rotationOfRobotInDegrees) {
    List<Pose2d> poses = new ArrayList<>();
    double timestamp = 0.0;
    for (String cameraName : cameraNames) {
      LimelightHelpers.SetRobotOrientation(
          cameraName, rotationOfRobotInDegrees, 0.0, 0.0, 0.0, 0.0, 0.0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

      if (llMeasurement != null && llMeasurement.tagCount > 0) {
        poses.add(llMeasurement.pose);
        timestamp += llMeasurement.timestampSeconds;
      }
    }
    if (!cameraNames.isEmpty()) {
      timestamp /= cameraNames.size();
    }
    lastTimeStamp = timestamp;

    return averagePose2ds(poses);
  }

  public Pose2d averagePose2ds(List<Pose2d> poses) {
    // This method averages multiple Pose2d objects to return a single Pose2d.
    // The averaging logic can be customized based on the requirements.

    if (poses.isEmpty()) {
      DogLog.log(
          "Robot/Subsystmes/LimeLightVision/TagsSeen", "NO TAGS SEEN RETURNING EMPTY POSE 2D");
      return new Pose2d(); // Return a default pose if no poses are provided
    }

    double xSum = 0;
    double ySum = 0;
    double rotationSum = 0;

    for (Pose2d pose : poses) {
      xSum += pose.getX();
      ySum += pose.getY();
      rotationSum += pose.getRotation().getRadians();
    }

    int count = poses.size();
    DogLog.log(
        "Robot/Subsystmes/LimeLightVision/TagsSeen",
        new Pose2d(xSum / count, ySum / count, new Rotation2d(rotationSum / count)));
    return new Pose2d(xSum / count, ySum / count, new Rotation2d(rotationSum / count));
  }

  public double getLastTimeStamp() {
    return lastTimeStamp;
  }
}
