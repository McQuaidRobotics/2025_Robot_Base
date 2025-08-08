package igknighters.commands.teleop;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.generated.knightshadeConsts;

public class TeleopSwerveHeadingCmd extends TeleopSwerveBaseCmd {
  private final double heading;
  private final SwerveRequest.FieldCentric m_driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
  PIDController rotationController = new PIDController(.5, 0.0, 0.0);

  public TeleopSwerveHeadingCmd(
      CommandSwerveDrivetrain swerve, DriverController controller, double heading) {
    super(swerve, controller);
    this.heading = heading;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    double omega = rotationController.calculate(swerve.getState().RawHeading.getDegrees(), heading);
    super.execute();
    Translation2d vt = translationStick();

    swerve.setControl(
        m_driveRequest
            .withVelocityX(vt.getX() * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond))
            .withVelocityY(vt.getY() * knightshadeConsts.kSpeedAt12Volts.in(MetersPerSecond))
            .withRotationalRate(omega));
  }
}
