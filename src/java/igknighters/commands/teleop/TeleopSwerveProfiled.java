package igknighters.commands.teleop;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import igknighters.controllers.DriverController;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.swerve.generated.TunerConstants;

public class TeleopSwerveProfiled extends TeleopSwerveBaseCmd {
  private final double detune;
  private final SwerveRequest.FieldCentric m_driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
          .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond)*.1)
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);

  public TeleopSwerveProfiled(
      CommandSwerveDrivetrain swerve, DriverController controller, double detune) {
    super(swerve, controller);
    this.detune = detune;
  }

  @Override
  public void execute() {
    super.execute();
    Translation2d vt = translationStick();

    swerve.setControl(
        m_driveRequest
            .withVelocityX(vt.getX() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * detune)
            .withVelocityY(vt.getY() * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * detune)
            .withRotationalRate(
                detune * RotationsPerSecond.of(0.75).in(RadiansPerSecond))
      );
  }
}
