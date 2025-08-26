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
    if (poses.isEmpty()) {
        DogLog.log("Robot/Subsystems/LimeLightVision/TagsSeen", "NO TAGS SEEN");
        return null; // safer than returning (0,0,0)
    }

    double xSum = 0.0, ySum = 0.0;
    double sinSum = 0.0, cosSum = 0.0;

    for (Pose2d pose : poses) {
        xSum += pose.getX();
        ySum += pose.getY();
        sinSum += Math.sin(pose.getRotation().getRadians());
        cosSum += Math.cos(pose.getRotation().getRadians());
    }

    int count = poses.size();
    double avgX = xSum / count;
    double avgY = ySum / count;
    Rotation2d avgRot = new Rotation2d(Math.atan2(sinSum / count, cosSum / count));

    Pose2d averaged = new Pose2d(avgX, avgY, avgRot);
    DogLog.log("Robot/Subsystems/LimeLightVision/TagsSeen", averaged);
    return averaged;
}

  public double getLastTimeStamp() {
    return lastTimeStamp;
  }
}
