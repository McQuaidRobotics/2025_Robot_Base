package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.generated.knightshadeConsts;
import wpilibExt.AllianceSymmetry;

public class TeleopSwerveTargetingCmd extends TeleopSwerveBaseCmd {

  private final Pose2d targetPose;
  private final SwerveRequest.FieldCentric m_driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
  private final PIDController rotationController = new PIDController(.1, 0.0, .0);

  public TeleopSwerveTargetingCmd(
      CommandSwerveDrivetrain swerve, DriverController controller, Pose2d targetPose) {
    super(swerve, controller);
    this.targetPose = targetPose;
    addRequirements(swerve);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(.0001);
  }

  @Override
  public void execute() {
    final var currentPose = swerve.getState().Pose;

    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();

    double desiredAngleRad = Math.atan2(dy, dx);
    double currentAngleRad = currentPose.getRotation().getRadians();

    // Wrap both angles explicitly

    double error = desiredAngleRad - currentAngleRad;

    DogLog.log(
        "Robot/Commands/Swerve/TeleopSwerveTargetingCmd/Desired Angle (deg)",
        Math.toDegrees(desiredAngleRad));
    DogLog.log(
        "Robot/Commands/Swerve/TeleopSwerveTargetingCmd/Current Angle (deg)",
        Math.toDegrees(currentAngleRad));
    DogLog.log(
        "Robot/Commands/Swerve/TeleopSwerveTargetingCmd/Wrapped Error (deg)",
        Math.toDegrees(error));

    double omega =
        rotationController.calculate(
            currentAngleRad * 180.0 / Math.PI, desiredAngleRad * 180 / Math.PI);

    DogLog.log("Robot/Commands/Swerve/TeleopSwerveTargetingCmd/PID Output", omega);

    Translation2d vt = translationStick();
    double allianceFlipper = 0.0;
    if (AllianceSymmetry.isBlue()) {
      allianceFlipper = 1.0;
    } else {
      allianceFlipper = -1.0;
    }

    swerve.setControl(
        m_driveRequest
            .withVelocityX(
                vt.getX() * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * allianceFlipper)
            .withVelocityY(
                vt.getY() * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * allianceFlipper)
            .withRotationalRate(omega));
  }
}
