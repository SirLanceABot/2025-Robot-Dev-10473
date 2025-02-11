package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Represents a Limelight to track AprilTags. */
public class Camera extends SensorLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    //INPUTS
    private DoubleEntry yawEntry;
    private NetworkTable cameraTable;
    private NetworkTableEntry botpose_orb_wpiblue; // Mega Tag 2

    //OUTPUTS
    private DoubleArrayEntry poseEntry;
    
    private String cameraName;
    private LimelightHelpers.PoseEstimate mt2PoseEstimate;
    private double[] poseArray = new double[3];

    private NetworkTable ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME); // custom table for AdvantageScope testing

    public Camera(String cameraName)
    {   
        super("Camera");
        this.cameraName = cameraName;
        
        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);

        poseEntry = ASTable.getDoubleArrayTopic(cameraName + " pose").getEntry(new double[3]);
        yawEntry = ASTable.getDoubleTopic("GyroYaw").getEntry(0.0);

        cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);
        botpose_orb_wpiblue = cameraTable.getEntry("botpose_orb_wpiblue");
        botpose_orb_wpiblue.setDefaultDoubleArray(new double[11]);

        mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + cameraName);
    }

    public LimelightHelpers.PoseEstimate getPoseEstimate()
    {
        LimelightHelpers.PoseEstimate limelightMeasurement = getPoseEstimate();
        return limelightMeasurement;
    }

    /** @return the MT2 robot pose with a blue driverstration origin*/
    public Pose2d getPose()
    {
        return mt2PoseEstimate.pose;

        // double[] bow = botpose_orb_wpiblue.getDoubleArray(new double[11]);
        // Translation2d translation = new Translation2d(bow[0], bow[1]);
        // Rotation2d rotation = new Rotation2d(bow[5]);
    }

    /** @return the timestamp calculated by LimelightHelpers */
    public double getTimestamp()
    {
        return mt2PoseEstimate.timestampSeconds;
    }

    /** @return the number of tags visible */
    public double getTagCount()
    {
        // return mt2PoseEstimate.tagCount;

        double[] t2d = ASTable.getEntry("t2d").getDoubleArray(new double[17]);

        return t2d[1];
    }

    /** @return the average distance from tags in meters */
    public double getAverageTagDistance()
    {
        return mt2PoseEstimate.avgTagDist;
    }

    public boolean isTargetFound()
    {
        return ASTable.getEntry("tv").getDouble(0.0) >= 0.5;
    }

    @Override
    public void periodic()
    {
        // readPeriodicInputs

        mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);


        // writePeriodicOutputs

        // LL publishes a 3D pose in a weird format, so to make it readable
        // we need to create our own double array and publish that
        
        poseArray[0] = mt2PoseEstimate.pose.getX();
        poseArray[1] = mt2PoseEstimate.pose.getY();
        poseArray[2] = mt2PoseEstimate.pose.getRotation().getDegrees();

        // put the pose from LL onto the Network Table so AdvantageScope can read it
        poseEntry.set(poseArray);        

        LimelightHelpers.SetRobotOrientation(cameraName, yawEntry.get(), 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}