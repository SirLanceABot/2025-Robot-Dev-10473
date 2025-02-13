package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public abstract class CameraLance
{
    public static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final StructPublisher<Pose3d> botpose_orb_wpiblue;

    CameraLance(String name)
    {
        var tableLogged = inst.getTable(name + "Logged");
        botpose_orb_wpiblue = tableLogged.getStructTopic("botpose_orb_wpiblue", Pose3d.struct).publish();
    }

    abstract Pose3d getPose3d();

    /**
     * Publish the limelight MegaTag2 blue 3d pose to NT; that's AdvantageScope format
     */
    public void publishPose3d()
    {
        botpose_orb_wpiblue.set(getPose3d());
    }

}
