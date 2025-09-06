package igknighters.subsystems.LimeLightVision.Helpers;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpersSimTools {

    public static void initializeSimData(String limelightName) {
        // Initialize all expected NetworkTables entries with zero/default values
        // Doubles
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("txnc").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tync").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ta").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tl").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("cl").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("getpipe").setDouble(0.0);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tid").setDouble(0.0);

        // Strings
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tcclass").setString("");
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tdclass").setString("");
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("getpipetype").setString("");
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("json").setString("{}"); // Empty JSON object
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tclass").setString("");

        // Double Arrays (6 elements for poses, 17 for t2d, 0 for rawfiducials/rawdetections initially)
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("t2d").setDoubleArray(new double[17]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_wpired").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_wpiblue").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_targetspace").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("camerapose_targetspace").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("targetpose_cameraspace").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("targetpose_robotspace").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("camerapose_robotspace").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tc").setDoubleArray(new double[0]); // Assuming target color is an empty array initially
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("rawfiducials").setDoubleArray(new double[0]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("rawdetections").setDoubleArray(new double[0]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_orb_wpiblue").setDoubleArray(new double[6]);
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_orb_wpired").setDoubleArray(new double[6]);

        // String Arrays
        NetworkTableInstance.getDefault().getTable(limelightName).getEntry("rawbarcodes").setStringArray(new String[0]);

        NetworkTableInstance.getDefault().flush();
    }
}
