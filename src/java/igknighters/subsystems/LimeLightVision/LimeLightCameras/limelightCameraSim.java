package igknighters.subsystems.LimeLightVision.LimeLightCameras;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class limelightCameraSim {
  // unfortunately, this is a placeholder class for the limelight camera simulation.
  // limelight does not provide a simulation API, so we cannot simulate the camera.
  // we will use photonvision for simulation instead.
  private final PhotonCameraSim photonCameraSim;
  private final PhotonCamera photonCamera;
  private final SimCameraProperties properties;

  public limelightCameraSim() {
    properties = new SimCameraProperties();
    properties.setCalibError(0.5, 0.02);
    properties.setFPS(43.0);
    properties.setAvgLatencyMs(20.0);
    properties.setLatencyStdDevMs(2.0);
    photonCamera = new PhotonCamera("Limelight");
    photonCameraSim = new PhotonCameraSim(photonCamera, properties, 0.12, 6.5);
    photonCameraSim.enableDrawWireframe(true);
  }
}
