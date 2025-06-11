package igknighters.commands;

import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class SwerveCommands {
  private final CommandSwerveDrivetrain swerve;
  public SwerveCommands(CommandSwerveDrivetrain drivetrain) {
    swerve = drivetrain;
  }
  
}
