package frc.robot.loggers;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public final class DataLogFile 
{
    private static NetworkTableInstance networkTableInstance;
    private static DataLog dataLog;

    private DataLogFile()
    {}

    public static void config()
    {
        // DataLogManager.start();
        DataLogManager.logNetworkTables(false);
        dataLog = DataLogManager.getLog();
        
        networkTableInstance = NetworkTableInstance.getDefault();
        DriverStation.startDataLog(dataLog, true);
        networkTableInstance.startEntryDataLog(dataLog, "/FMSInfo", "NT:/FMSInfo");
        networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.NETWORK_TABLE_NAME, "NT:/" + Constants.NETWORK_TABLE_NAME);
        networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME, "NT:/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.Camera.CAMERA_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_BOT_POSE);
        networkTableInstance.startEntryDataLog(dataLog, "/SmartDashboard", "NT:/SmartDashboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/Shuffleboard", "NT:/Shuffleboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/LiveWindow", "NT:/LiveWindow");
        networkTableInstance.startConnectionDataLog(dataLog, "NTConnection");
    }
}
