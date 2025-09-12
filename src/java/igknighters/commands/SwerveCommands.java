package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import wpilibExt.AllianceSymmetry;

public class SwerveCommands {

  public static Command zeroGyro(CommandSwerveDrivetrain swerve) {
    return Commands.either(
        Commands.runOnce(
            () ->
                swerve.resetPose(
                    new Pose2d(
                        swerve.getState().Pose.getX(),
                        swerve.getState().Pose.getY(),
                        new Rotation2d(0.0)))),
        Commands.runOnce(
            () ->
                swerve.resetPose(
                    new Pose2d(
                        swerve.getState().Pose.getX(),
                        swerve.getState().Pose.getY(),
                        new Rotation2d(Math.PI)))),
        AllianceSymmetry::isBlue);
  }

  public static Pose2d getPose(CommandSwerveDrivetrain swerve) {
    return swerve.getState().Pose;
  }
}
