package igknighters.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import wpilibExt.AllianceSymmetry;

public class SwerveCommands {

  public static Command zeroGyro(CommandSwerveDrivetrain swerve) {
    return Commands.either(
        Commands.runOnce(() -> swerve.resetRotation(new Rotation2d(0.0))),
        Commands.runOnce(() -> swerve.resetRotation(new Rotation2d(Math.PI))),
        AllianceSymmetry::isBlue);
  }
}
