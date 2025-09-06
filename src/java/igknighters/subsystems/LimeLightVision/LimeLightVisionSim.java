package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;

public class LimeLightVisionSim extends LimeLights {
  @Override
  public Pose2d getRobotPoseFromVision(
      double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {
    return null;
  }

  @Override
  public double getLastTimeStamp() {
    return 0;
  }
  
}
