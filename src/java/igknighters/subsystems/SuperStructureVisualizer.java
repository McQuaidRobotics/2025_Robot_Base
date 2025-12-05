package igknighters.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import igknighters.constants.ConstValues;
import igknighters.constants.ConstValues.kRobotCollisionGeometry;

public class SuperStructureVisualizer {
    private final Mechanism2d superstructure;
    private final MechanismRoot2d pivotRoot;
    private final MechanismLigament2d telescope, shooter, intake;
    private final double MAX_TELESCOPE_LENGTH = ConstValues.kStem.kTelescope.MAX_METERS;
    private final double UMBRELLA_LENGTH = kRobotCollisionGeometry.UMBRELLA_LENGTH;
    private final double WIDTH = 10;

    public SuperStructureVisualizer() {

        superstructure = new Mechanism2d(1.5, 1.5);
        pivotRoot = superstructure.getRoot("Pivot", .75, 0);
        telescope = pivotRoot.append(new MechanismLigament2d("telescope", 1.0, 0.0));
        shooter = telescope.append(new MechanismLigament2d("wrist", UMBRELLA_LENGTH / 2, 0.0));

        intake = shooter.append(new MechanismLigament2d("intake", UMBRELLA_LENGTH / 2, 0.0));

        superstructure.setBackgroundColor(new Color8Bit(Color.kBlack));
        telescope.setColor(new Color8Bit(Color.kBlue));
        shooter.setColor(new Color8Bit(Color.kRed));
        intake.setColor(new Color8Bit(Color.kRed));

        shooter.setLineWeight(WIDTH);
        intake.setLineWeight(WIDTH);
        telescope.setLineWeight(WIDTH);

        SmartDashboard.putData("SUPERSTRUCTURE", superstructure);
    }

    public Color8Bit interpolateColor(Color8Bit color1, Color8Bit color2, double fraction) {
        int r = (int) (color1.red * (1 - fraction) + color2.red * fraction);
        int g = (int) (color1.green * (1 - fraction) + color2.green * fraction);
        int b = (int) (color1.blue * (1 - fraction) + color2.blue * fraction);
        return new Color8Bit(r, g, b);
    }

    public void updateFromStem(double pivotAngle, double telescopeLength, double wristAngle) {

        telescope.setAngle(pivotAngle);
        telescope.setLength(telescopeLength / MAX_TELESCOPE_LENGTH);
        shooter.setAngle(-wristAngle);
    }

    public void updateFromIntake(double intakeRPM) {
        double intakeFraction = Math.min(Math.abs(intakeRPM) / 5000.0, 1.0);

        Color8Bit intakeColor =
                interpolateColor(
                        new Color8Bit(Color.kRed), new Color8Bit(Color.kGreen), intakeFraction);

        intake.setColor(intakeColor);
    }

    public void updateFromShooter(double shooterRPM) {
        double shooterFraction = Math.min(Math.abs(shooterRPM) / 10000.0, 1.0);

        Color8Bit shooterColor =
                interpolateColor(
                        new Color8Bit(Color.kRed), new Color8Bit(Color.kGreen), shooterFraction);

        shooter.setColor(shooterColor);
    }

    private static class SingletonHelper {
        private static final SuperStructureVisualizer INSTANCE = new SuperStructureVisualizer();
    }

    public static SuperStructureVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }
}
