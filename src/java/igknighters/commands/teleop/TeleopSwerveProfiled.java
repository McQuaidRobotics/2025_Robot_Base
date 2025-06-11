package igknighters.commands.teleop;

import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class TeleopSwerveProfiled {
  private final CommandSwerveDrivetrain swerve;
  public TeleopSwerveProfiled(CommandSwerveDrivetrain drivetrain) {
    swerve = drivetrain;
  }
  
}
