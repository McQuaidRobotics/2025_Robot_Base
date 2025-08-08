package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;

public class Subsystems {
  public final CommandSwerveDrivetrain swerve;
  public final LimeLightVision vision;
  public final ExclusiveSubsystem[] lockedResources;
  public final SharedSubsystem[] locklessResources;

  public Subsystems(CommandSwerveDrivetrain drivetrain, LimeLightVision vision) {
    this.swerve = drivetrain;
    this.vision = vision;
    this.lockedResources = new ExclusiveSubsystem[] {this.swerve};
    this.locklessResources = new SharedSubsystem[] {vision};
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
