package frc.robot.sensors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Basis of cameras that supply the 3-D pose of the robot
 * 
 * <p>Defines required methods for cameras.
 * 
 * <p>Provides method to put the robot 3-D pose wrt BLUE origin on NetworkTables for display purposes.
 */
public abstract class CameraLance
{
    public static final NetworkTableInstance NTinstance = NetworkTableInstance.getDefault();
    private final StructPublisher<Pose3d> botpose_orb_wpiblue;

    CameraLance(String name)
    {
        var tableLogged = NTinstance.getTable(name + "Logged");
        botpose_orb_wpiblue = tableLogged.getStructTopic("botpose_orb_wpiblue", Pose3d.struct).publish();
    }

    abstract boolean isFresh();
    abstract Pose3d getPose3d();
    abstract Pose2d getPose2d();
    abstract double getTX();
    abstract double getTY();
    abstract void update();

    /**
     * Publish the limelight MegaTag2 blue 3d pose to NT; that's AdvantageScope format
     */
    public void publishPose3d()
    {
        if (isFresh())
        {
            botpose_orb_wpiblue.set(getPose3d());
        }
    }
}
