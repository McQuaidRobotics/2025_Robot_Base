package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import igknighters.subsystems.LimeLightVision.LimeLights;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.umbrella.Umbrella;
import monologue.Logged;

public class Subsystems {
  public final CommandSwerveDrivetrain swerve;
  public final LimeLights vision;
  public final Led led;
  public final Umbrella umbrella;
  public final Stem stem;
  public final ExclusiveSubsystem[] lockedResources;
  public final SharedSubsystem[] locklessResources;

  public Subsystems(
      CommandSwerveDrivetrain drivetrain,
      LimeLights vision,
      Led led,
      Umbrella umbrella,
      Stem stem) {
    this.swerve = drivetrain;
    this.vision = vision;
    this.led = led;
    this.umbrella = umbrella;
    this.stem = stem;

    this.lockedResources = new ExclusiveSubsystem[] {this.swerve, led, umbrella, stem};
    this.locklessResources = new SharedSubsystem[] {vision};

    CommandScheduler.getInstance().registerSubsystem(this.lockedResources);
    for (SharedSubsystem subsystem : this.locklessResources) {
      CommandScheduler.getInstance()
          .registerSubsystem(
              new Subsystem() {
                @Override
                public void periodic() {
                  subsystem.periodic();
                }

                @Override
                public String getName() {
                  return subsystem.getName();
                }
              });
    }
  }

  public static interface ExclusiveSubsystem extends Subsystem, Logged {}

  public static interface SharedSubsystem {
    default void periodic() {}

    default void simulationPeriodic() {}

    default String getName() {
      return this.getClass().getSimpleName();
    }
  }
}
