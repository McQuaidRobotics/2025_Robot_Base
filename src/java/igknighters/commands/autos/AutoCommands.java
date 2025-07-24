package igknighters.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public class AutoCommands {

  protected final CommandSwerveDrivetrain swerve;
  protected final AutoFactory autoFactory;

  public AutoCommands(Subsystems subsystems, AutoFactory factory) {
    this.swerve = subsystems.swerve;
    this.autoFactory = factory;
  }

  protected void logAutoEvent(String message, String event) {
    DogLog.log("Robot/Commands/Autos", message + " is " + event);
  }

  protected Command loggedCmd(Command command) {
    return new WrapperCommand(command) {
      @Override
      public void initialize() {
        logAutoEvent(this.getName(), "Started");
        super.initialize();
      }

      @Override
      public void end(boolean interrupted) {
        super.end(interrupted);
        logAutoEvent(this.getName(), "Finished");
      }
    };
  }

  protected Boolean withinTolerance(Pose2d pose, Pose2d target, double tolerance) {
    return pose.getTranslation().getDistance(target.getTranslation()) < tolerance
        && Math.abs(pose.getRotation().getDegrees() - target.getRotation().getDegrees())
            < tolerance;
  }

  protected double findSpeed(ChassisSpeeds speeds) {
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  protected Trigger movingSlowerThan(CommandSwerveDrivetrain swerve, double speed) {
    return new Trigger(() -> findSpeed(swerve.getState().Speeds) < speed);
  }

  protected class GenericAuto {
    private final AutoRoutine routine;
    private final ParallelCommandGroup headCommand = new ParallelCommandGroup();
    private final SequentialCommandGroup bodyCommand = new SequentialCommandGroup();
    private final boolean leftSide;
    private boolean trajectorybeenadded = false;

    private GenericAuto(AutoRoutine routine, boolean leftSide) {
      this.routine = routine;
      this.leftSide = leftSide;
    }

    private AutoTrajectory getTrajectory(Waypoints start, Waypoints end) {
      if (leftSide) {
        return routine.trajectory(start.to(end));
      } else {
        Trajectory<?> rawTraj = autoFactory.cache().loadTrajectory(start.to(end)).orElseThrow();
        rawTraj = rawTraj.flipped();
        return routine.trajectory(rawTraj);
      }
    }

    private Command finishAlignment(AutoTrajectory trajectory, double distOffset) {
      if (trajectory.getFinalPose().isPresent()) {
        Supplier<Command> cmdSup =
            () -> {
              final Pose2d finalPose =
                  trajectory
                      .getFinalPose()
                      .get()
                      .plus(new Transform2d(distOffset, 0, Rotation2d.kZero));
              return loggedCmd(
                  SwerveCommands.moveToSimple(swerve, finalPose)
                      .until(
                          () ->
                              withinTolerance(SwerveCommands.getPose(swerve), finalPose, 0.1)
                                  && movingSlowerThan(swerve, .08).getAsBoolean()));
            };
        return Commands.defer(cmdSup, Set.of(swerve));
      } else {
        DriverStation.reportError("NO FINAL POSE IN THE AUTO ROUTINE", false);
        return Commands.none();
      }
    }

    public GenericAuto addDrivingTrajectory(Waypoints... waypoints) {
      headCommand.addCommands(getTrajectory(waypoints[0], waypoints[1]).resetOdometry());
      for (int i = 0; i < waypoints.length - 2; i += 2) {
        bodyCommand.addCommands(
            getTrajectory(waypoints[i], waypoints[i + 1]).cmd(),
            Commands.waitSeconds(3.0),
            getTrajectory(waypoints[i + 1], waypoints[i + 2]).cmd(),
            Commands.waitSeconds(3.0));
      }
      return this;
    }

    public Command build() {
      final AtomicBoolean flag = new AtomicBoolean(false);
      headCommand.addCommands(Commands.print(bodyCommand.getRequirements().toString()));
      bodyCommand.addCommands(new ScheduleCommand(Commands.runOnce(() -> flag.set(true))));
      routine
          .active()
          .onTrue(
              headCommand
                  .andThen(new ScheduleCommand(bodyCommand))
                  .withName(routine.toString() + "_AutoHead"));
      return routine.cmd(flag::get);
    }
  }

  protected GenericAuto newAuto(String name, boolean leftSide) {
    return new GenericAuto(autoFactory.newRoutine(name), leftSide);
  }
}
