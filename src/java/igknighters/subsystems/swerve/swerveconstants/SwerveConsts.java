package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.RobotController;

public class SwerveConsts {

  String robotSerialNumber;

  public enum Robots {
    DEMO_BOT,
    COMPETITION_BOT,
    UNKNOWN
  };

  private String DEMO_BOT_SERIAL_NUMBER = "TBD";
  private String COMPETION_BOT_SERIAL_NUMBER = "IT DOESNT EXIST YET";

  public Robots getRobot() {
    robotSerialNumber = RobotController.getSerialNumber();
    if (robotSerialNumber == DEMO_BOT_SERIAL_NUMBER) {
      return Robots.DEMO_BOT;
    } else if (robotSerialNumber == COMPETION_BOT_SERIAL_NUMBER) {
      return Robots.COMPETITION_BOT;
    } else {
      return Robots.UNKNOWN;
    }
  }
  /**
   * Get the swerve constants for the current robot
   * the way it is configured curently it will return DemoBotConsts for unknown robots
   * @return CommonSwerveConsts for the current robot
   */
  public CommonSwerveConsts getSwerveConsts() {
    Robots robot = getRobot();
    if (robot.equals(Robots.DEMO_BOT)) {
      return new DemoBotConsts();
    } else if (robot.equals(Robots.COMPETITION_BOT)) {
      return new knightshadeConsts();
    } else {
      return new DemoBotConsts();
    }
  }
}
