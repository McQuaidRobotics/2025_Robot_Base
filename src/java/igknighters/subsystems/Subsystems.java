package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
  public final CommandSwerveDrivetrain swerve;
  public final ExclusiveSubsystem[] lockedResources;
  public final SharedSubsystem[] locklessResources;

  public Subsystems(CommandSwerveDrivetrain drivetrain) {
    this.swerve = drivetrain;
    this.lockedResources = new ExclusiveSubsystem[] {this.swerve};
    this.locklessResources = new SharedSubsystem[]{};
  }

  

  public static interface ExclusiveSubsystem extends Subsystem {}

  public static interface SharedSubsystem {
    default void periodic() {}

    default void simulationPeriodic() {}

    default String getName() {
      return this.getClass().getSimpleName();
    }
  }
}
