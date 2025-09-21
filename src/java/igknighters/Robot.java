// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import igknighters.commands.autos.AutoRoutines;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightVisionConstants;
import igknighters.subsystems.LimeLightVision.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.LimeLightVisionSim;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.swerve.generated.knightshadeConsts;
import monologue.LogSink;
import monologue.Monologue;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final AutoFactory autoFactory;
  public final AutoChooser autoChooser = new AutoChooser();
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  private final DriverController driverController = new DriverController(0);

  private final Telemetry logger =
      new Telemetry(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond));

  public final Subsystems subsytems;

  private final boolean kUseLimelight = true;

  public Robot() {
    subsytems =
        (Robot.isReal())
            ? new Subsystems(
                knightshadeConsts.createDrivetrain(),
                new LimeLightVisionReal(LimelightVisionConstants.backRight))
            : new Subsystems(knightshadeConsts.createDrivetrain(), new LimeLightVisionSim());
    subsytems.swerve.setDefaultCommand(
        new TeleopSwerveWithDetune(subsytems.swerve, driverController, .3));
    subsytems.swerve.registerTelemetry(logger::telemeterize);
    driverController.bind(subsytems);
    autoFactory = subsytems.swerve.createAutoFactory();
    final var routines = new AutoRoutines(subsytems, autoFactory);
    AutoRoutines.addCmd(autoChooser, "ZOOOOOOOOMMMMMMM", routines::driveAround);
    autoChooser.addCmd("TRAJECTORY TEST", routines.trajTest("Straight"));
    SmartDashboard.putData("AUTO CHOOSER", autoChooser);
    // RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
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
        subsytems.swerve.addVisionMeasurement(currentPose, subsytems.vision.getLastTimeStamp());
      }
    }
  }

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Command autoCmd = autoChooser.selectedCommand();
    String msg = "---- Starting auto command: " + autoCmd.getName() + " ----";
    if (false) System.out.println(msg);
    Monologue.log("AutoEvent", msg);
    scheduler.schedule(autoCmd);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
  }

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
  public void simulationPeriodic() {}

  private void setupAutoChooser() {
    Monologue.publishSendable("/Choosers/AutoChooser", autoChooser, LogSink.NT);
    final StringSubscriber sub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/Choosers/AutoChooser/selected")
            .subscribe(
                "",
                PubSubOption.pollStorage(1),
                PubSubOption.periodic(0.5),
                PubSubOption.sendAll(true),
                PubSubOption.keepDuplicates(false));
    this.addPeriodic(
        () -> {
          var queue = sub.readQueueValues();
          if (queue.length > 0) {
            System.out.println("AutoChooser selected: " + queue[0]);
            autoChooser.select(queue[0]);
          }
        },
        kDefaultPeriod,
        0.01);
  }
}
