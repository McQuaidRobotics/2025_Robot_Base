package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import igknighters.commands.led.LEDCommands;
import igknighters.commands.led.LEDCommands.LEDSection;
import igknighters.commands.stem.StemCommands;
import igknighters.commands.teleop.TeleopSwerveReverseTargetingCmd;
import igknighters.commands.umbrella.UmbrellaCommands;
import igknighters.constants.ConstValues.kControls;
import igknighters.constants.ConstValues.kStem.kTelescope;
import igknighters.constants.ConstValues.kStem.kWrist;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FieldConstants;
import igknighters.controllers.DriverController;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.led.LedUtil;
import igknighters.subsystems.stem.Stem;
import igknighters.subsystems.stem.StemPosition;
import igknighters.subsystems.swerve.CommandSwerveDrivetrain;
import igknighters.subsystems.umbrella.Umbrella;
import igknighters.util.geom.AllianceFlip;

public class HigherOrderCommands {

    public static Command intakeGamepiece(Stem stem, Umbrella umbrella, Led led) {
        return Commands.race(
                        StemCommands.holdAt(stem, StemPosition.INTAKE),
                        Commands.idle()
                                .until(
                                        () -> {
                                            var pose = stem.getStemPosition();
                                            return pose.wristRads
                                                            > (StemPosition.INTAKE.wristRads
                                                                            + kWrist.MIN_ANGLE)
                                                                    / 2.0
                                                    && pose.telescopeMeters > kTelescope.MIN_METERS;
                                        })
                                .andThen(
                                        UmbrellaCommands.intake(umbrella)
                                                .until(() -> umbrella.holdingGamepiece())))
                .andThen(
                        new ScheduleCommand(
                                StemCommands.holdAt(stem, StemPosition.STOW),
                                LEDCommands.run(
                                        led,
                                        new LEDSection(
                                                0,
                                                0,
                                                LedUtil.makeFlash(Color.kGreen, 0.2),
                                                40,
                                                "INTAKE"))))
                .withName("Intake");
    }

    public static Command aim(
            CommandSwerveDrivetrain swerve, Stem stem, DriverController controller) {
        return Commands.parallel(
                        new TeleopSwerveReverseTargetingCmd(
                                swerve,
                                controller,
                                new Pose2d(
                                        AllianceFlip.isBlue()
                                                ? new Translation2d(
                                                        FieldConstants.SPEAKER.getX(),
                                                        FieldConstants.SPEAKER.getY())
                                                : new Translation2d(
                                                        AllianceFlip.flipTranslation(
                                                                        FieldConstants.SPEAKER)
                                                                .getX(),
                                                        AllianceFlip.flipTranslation(
                                                                        FieldConstants.SPEAKER)
                                                                .getY()),
                                        new Rotation2d()),
                                DrivingSharedState.getInstance().kP,
                                DrivingSharedState.getInstance().kI,
                                DrivingSharedState.getInstance().kD),
                        StemCommands.aimAtSpeaker(
                                stem,
                                false,
                                () -> swerve.getState().Pose,
                                () -> swerve.getState().Speeds))
                .withName("Aim");
    }

    public static Command aimNotePass(
            CommandSwerveDrivetrain swerve, Stem stem, DriverController controller) {
        return Commands.parallel(
                new TeleopSwerveReverseTargetingCmd(
                        swerve,
                        controller,
                        new Pose2d(
                                AllianceFlip.isBlue()
                                        ? new Translation2d(
                                                kControls.PASS_LAND_LOCATION.getX(),
                                                kControls.PASS_LAND_LOCATION.getY())
                                        : new Translation2d(
                                                AllianceFlip.flipTranslation(
                                                                kControls.PASS_LAND_LOCATION)
                                                        .getX(),
                                                AllianceFlip.flipTranslation(
                                                                kControls.PASS_LAND_LOCATION)
                                                        .getY()),
                                new Rotation2d()),
                        DrivingSharedState.getInstance().kP,
                        DrivingSharedState.getInstance().kI,
                        DrivingSharedState.getInstance().kD),
                StemCommands.aimAtPassPoint(
                        stem,
                        kControls.PASS_LAND_LOCATION,
                        false,
                        // localizer::pose
                        () -> swerve.getState().Pose));
    }

    public static class ShootSequences {
        private static double targetRpm(Umbrella umbrella) {
            return (umbrella.getShooterTargetSpeed() / 60.0) * (2.0 * Math.PI);
        }

        public static Command shoot(Stem stem, Umbrella umbrella) {
            return Commands.deadline(
                            UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella)),
                            StemCommands.holdStill(stem))
                    .withName("Shoot");
        }

        public static Command autoAimShoot(
                CommandSwerveDrivetrain swerve,
                Stem stem,
                Umbrella umbrella,
                DriverController controller) {
            return Commands.parallel(
                            HigherOrderCommands.aim(swerve, stem, controller),
                            UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella)))
                    .until(() -> controller.leftTrigger(true).getAsDouble() < 0.5)
                    .withName("AutoAimShoot");
        }

        public static Command autoAimPassShoot(
                CommandSwerveDrivetrain swerve,
                Stem stem,
                Umbrella umbrella,
                DriverController controller) {
            return Commands.parallel(
                            HigherOrderCommands.aimNotePass(swerve, stem, controller),
                            UmbrellaCommands.shoot(umbrella, () -> targetRpm(umbrella)))
                    .withName("AutoAimShoot");
        }

        public static Command ampShoot(Stem stem, Umbrella umbrella) {
            return Commands.sequence(
                            StemCommands.moveTo(stem, StemPosition.AMP_SCORE, 1.4),
                            umbrella.run(
                                            () -> {
                                                umbrella.spinupShooter(kControls.SHOOTER_RPM);
                                                umbrella.runIntakeAt(-1.0, true);
                                            })
                                    .withTimeout(0.3),
                            StemCommands.moveTo(stem, StemPosition.AMP_SAFE, 1.5))
                    .withName("AmpShoot");
        }
    }
}
