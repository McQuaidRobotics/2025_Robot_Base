// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.SubsystemTriggers;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.DrivingSharedState;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightVisionConstants;
import igknighters.subsystems.LimeLightVision.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.LimeLightVisionSim;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;
import igknighters.subsystems.umbrella.Umbrella;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    private final SubsystemTriggers subsystemTriggers = new SubsystemTriggers();

    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsytems;

    private final boolean kUseLimelight = true;

    private final SwerveConsts swerveConstGetter = new SwerveConsts();

    private final CommonSwerveConsts swerveConsts = swerveConstGetter.getSwerveConsts();

    private final Telemetry logger;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 0.6);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    public Robot() {
        if (Robot.isReal()) {
            subsytems =
                    new Subsystems(
                            swerveConsts.createDrivetrain(),
                            new LimeLightVisionReal(LimelightVisionConstants.backLeft),
              new Led(40, 1),
              new Umbrella(),
              new Stem());
        } else {
            subsytems =
          new Subsystems(
              swerveConsts.createDrivetrain(),
              new LimeLightVisionSim(),
              new Led(40, 1),
              new Umbrella(),
              new Stem());
        }
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, .8));

        logger = new Telemetry(swerveConsts.getMaxSpeedMetersPerSecond(), subsytems);
        subsytems.swerve.registerTelemetry(logger::telemeterize);
        driverController.bind(subsytems);
        autoFactory = subsytems.swerve.createAutoFactory();
        final var routines = new AutoRoutines(subsytems, autoFactory);
        AutoRoutines.addCmd(autoChooser, "ZOOOOOOOOMMMMMMM", routines::driveAround);
        autoChooser.addCmd("TRAJECTORY TEST", routines.trajTest("Straight"));
        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
        subsystemTriggers.SetupTriggers(subsytems.led);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (kUseLimelight) {
            var driveState = subsytems.swerve.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            Pose2d currentPose =
                    subsytems.vision.getRobotPoseFromVision(headingDeg, omegaRps, 0, 0, 0, 0);
            if (currentPose != null) {
                subsytems.swerve.addVisionMeasurement(
                        currentPose, subsytems.vision.getLastTimeStamp());
            }
        }
    }

    @Override
    public void disabledInit() {
        scheduler.cancelAll();
        subsytems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsytems.swerve, driverController, detune.value()));
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());

        driverController.bind(subsytems);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        Command autoCmd = autoChooser.selectedCommand();
        String msg = "---- Starting auto command: " + autoCmd.getName() + " ----";
        DogLog.log("AutoEvent", msg);
        scheduler.schedule(autoCmd);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        scheduler.cancelAll();
    }

  // private void setupLogging() {
  //   WatchdogSilencer.silence(this, "m_watchdog");
  //   WatchdogSilencer.silence(scheduler, "m_watchdog");

  //   DriverStation.silenceJoystickConnectionWarning(true);

  //   // turn off auto logging for signal logger, doesn't get us any info we need
  //   if (isReal()) {
  //     SignalLogger.enableAutoLogging(false);
  //   }

  //   if (true) {
  //     // setup monologue with lazy logging and no datalog prefix
  //     // robot is the root object
  //     Monologue.setupMonologue(
  //         this,
  //         "/Robot",
  //         new MonologueConfig().withOptimizeBandwidth(DriverStation::isFMSAttached));
  //   } else {
  //     // used for tests and CI, does not actually log anything but asserts the logging is setup
  //     // // mostly correct
  //     // Monologue.setupMonologueDisabled(this, "/Robot", true);
  //   }

  //   // Monologue.capture("Tracer", NetworkTableInstance.getDefault().getTable("Tracer"));

  //   // logs build data to the datalog
  //   final String meta = "/BuildData/";
  //   Monologue.log(meta + "RuntimeType", getRuntimeType().toString());
  //   Monologue.log(meta + "ProjectName", BuildConstants.MAVEN_NAME);
  //   Monologue.log(meta + "BuildDate", BuildConstants.BUILD_DATE);
  //   Monologue.log(meta + "GitSHA", BuildConstants.GIT_SHA);
  //   Monologue.log(meta + "GitDate", BuildConstants.GIT_DATE);
  //   Monologue.log(meta + "GitBranch", BuildConstants.GIT_BRANCH);
  //   switch (BuildConstants.DIRTY) {
  //     case 0:
  //       Monologue.log(meta + "GitDirty", "All changes committed");
  //       break;
  //     case 1:
  //       Monologue.log(meta + "GitDirty", "Uncomitted changes");
  //       break;
  //     default:
  //       Monologue.log(meta + "GitDirty", "Unknown");
  //       break;
  //   }
  //   Monologue.log(meta + "Debug", false);
  //   Monologue.log(meta + "Demo", isDemo());

  //   BiConsumer<Command, Boolean> logCommandFunction =
  //       (Command command, Boolean active) -> {
  //         Monologue.log("Commands/" + command.getName(), active);
  //       };
  //   scheduler.onCommandInitialize(
  //       (Command command) -> {
  //         logCommandFunction.accept(command, true);
  //       });
  //   scheduler.onCommandFinish(
  //       (Command command) -> {
  //         logCommandFunction.accept(command, false);
  //       });
  //   scheduler.onCommandInterrupt(
  //       (Command command) -> {
  //         logCommandFunction.accept(command, false);
  //       });

  //   Monologue.getWpilog().get().flush();
  // }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        for (var subsystem : subsytems.locklessResources) {
            subsystem.simulationPeriodic();
        }
  }

  public static boolean isDemo() {
    return true;
    }
}
