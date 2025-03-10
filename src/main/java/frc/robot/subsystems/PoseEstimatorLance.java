package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
// import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.sensors.CameraLL;
import frc.robot.sensors.GyroLance;

/**
 * @author Robbie Frank
 * @author Mason Bellinger
 * @author Brady Woodard
 * @author Aditya Yadav
 */
public class PoseEstimatorLance extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    


    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here
    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    
    private final GyroLance gyro;
    private final Drivetrain drivetrain;
    private final CameraLL camera;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final NetworkTable ASTable;
    private final DoubleArrayEntry poseEstimatorEntry;

    // private boolean isRight;

    private Pose2d estimatedPose = new Pose2d();
    private Pose2d estimatedOdometryPose = new Pose2d();

    // Pose2d poseA = new Pose2d();
    // Pose2d poseB = new Pose2d();

    // StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    //         .getStructTopic("MyPose", Pose2d.struct).publish();
    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
    private final StructPublisher<Pose2d> estimatedPosePublisher = networkTable
            .getStructTopic("EstimatedPose", Pose2d.struct).publish();

    // StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    //         .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    private final double[] defaultValues = {0.0, 0.0, 0.0};
    // private final double MAX_TARGET_DISTANCE = 5.0; // meters

    // private final HashMap<Integer, Pose2d> rightSideMap = new HashMap<Integer, Pose2d>();
    // private final HashMap<Integer, Pose2d> leftSideMap = new HashMap<Integer, Pose2d>();
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final List<AprilTag> aprilTagList = aprilTagFieldLayout.getTags();

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new PoseEstimator. 
     * 
     * tracks the robot's pose using data from the gyro, drivetrain, and the camera
     */
    public PoseEstimatorLance(GyroLance gyro, Drivetrain drivetrain, CameraLL camera)
    {
        super("PoseEstimator");
        System.out.println("  Constructor Started:  " + fullClassName);

        this.gyro = gyro;
        this.drivetrain = drivetrain;
        this.camera = camera;

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        // periodicData.odometryEntry = ASTable.getDoubleArrayTopic("Odometry").getEntry(defaultValues);
        poseEstimatorEntry = ASTable.getDoubleArrayTopic("PoseEstimator").getEntry(defaultValues);
        
        if(drivetrain != null && gyro != null)
        {
            poseEstimator = new DifferentialDrivePoseEstimator(
                drivetrain.getKinematics(),
                gyro.getRotation2d(),
                drivetrain.getLeftLeaderDistance(),
                drivetrain.getRightLeaderDistance(),
                drivetrain.getPose(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(999.0, 999.0, Units.degreesToRadians(999.0))
            );
        }
        else
        {
            poseEstimator = null;
        }

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public AprilTag getNearestTag()
    {
        AprilTag nearestTag = null;
        double min = 100.0;
        double dist;

        for(AprilTag tag : aprilTagList)
        {
            dist = tag.pose.getTranslation().toTranslation2d().getDistance(getEstimatedPose().getTranslation());

            if(dist < min)
            {
                min = dist;
                nearestTag = tag;
            }
        }
        return nearestTag;
    }

    /**
     * @return
     * the angle of the nearest april tag
     */
    public double getAngleOfNearestAprilTag()
    {
        AprilTag tag = getNearestTag();

        return tag.pose.getRotation().toRotation2d().getDegrees();
    }

    /**
     * @return
     * the angle the robot needs to be parrallel to the closest side of the reef
     */
    public double getAngleParallelToAprilTag()
    {
        return MathUtil.inputModulus(getAngleOfNearestAprilTag() - 90.0, 0.0, 360.0);
    }

    /**
     * @return
     * the angle the robot needs to be perpendicular to the closest side of the reef
     */
    public double getAnglePerpendicularToAprilTag()
    {
        return MathUtil.inputModulus(getAngleOfNearestAprilTag() + 180.0, 0.0, 360.0);
    }

    /**
     * @param pose
     * the pose of the robot
     * 
     * resets the PoseEstimator by reseting the gyro and drivetrain position
     */
    public void resetPoseEstimator(Pose2d pose)
    {
        if(gyro != null && drivetrain != null)
        {
            poseEstimator.resetPosition(
                gyro.getRotation2d(),
                drivetrain.getLeftLeaderDistance(),
                drivetrain.getRightLeaderDistance(),
                pose
            );
        }
    }



    // public void fillMaps()
    // {
    //     //Red Left
    //     leftSideMap.put( 6,  new Pose2d( new Translation2d(13.6102433, 3.1393908 ) , new Rotation2d(Math.toRadians(120.0)  )));
    //     leftSideMap.put( 7,  new Pose2d( new Translation2d(14.016898, 3.8659 )     , new Rotation2d(Math.toRadians(180.0)  )));
    //     leftSideMap.put( 8,  new Pose2d( new Translation2d(13.660146, 4.8824092 )  , new Rotation2d(Math.toRadians(-120.0) )));
    //     leftSideMap.put( 9,  new Pose2d( new Translation2d(12.457658, 4.8827472 )  , new Rotation2d(Math.toRadians(-60.0)  )));
    //     leftSideMap.put( 10, new Pose2d( new Translation2d(12.100906, 3.8659 )     , new Rotation2d(Math.toRadians(0.0)    )));
    //     leftSideMap.put( 11, new Pose2d( new Translation2d(12.5075747, 3.1393908 ) , new Rotation2d(Math.toRadians(60.0)   )));

    //     //Blue Left
    //     leftSideMap.put( 17, new Pose2d( new Translation2d(3.888206, 3.1693908 )   , new Rotation2d(Math.toRadians(60.0)   )));
    //     leftSideMap.put( 18, new Pose2d( new Translation2d(3.5312, 4.1859 )        , new Rotation2d(Math.toRadians(0.0)    )));
    //     leftSideMap.put( 19, new Pose2d( new Translation2d(3.9381227, 4.9124092 )  , new Rotation2d(Math.toRadians(-60.0)  )));
    //     leftSideMap.put( 20, new Pose2d( new Translation2d(5.0405233, 4.9124092 )  , new Rotation2d(Math.toRadians(-120.0) )));
    //     leftSideMap.put( 21, new Pose2d( new Translation2d(5.447446, 4.1859 )      , new Rotation2d(Math.toRadians(180.0)  )));
    //     leftSideMap.put( 22, new Pose2d( new Translation2d(5.06044, 3.1693908 )    , new Rotation2d(Math.toRadians(120.0)  )));

    //     //Red Right
    //     rightSideMap.put(6,  new Pose2d( new Translation2d(13.660146, 3.139308)    , new Rotation2d(Math.toRadians(120.0) )));
    //     rightSideMap.put(7,  new Pose2d( new Translation2d(14.016898, 4.1859 )     , new Rotation2d(Math.toRadians(180.0) )));
    //     rightSideMap.put(8,  new Pose2d( new Translation2d(13.6102293, 4.9124092 ) , new Rotation2d(Math.toRadians(-120.0))));
    //     rightSideMap.put(9,  new Pose2d( new Translation2d(12.5075747, 4.9124092 ) , new Rotation2d(Math.toRadians(-60.0) )));
    //     rightSideMap.put(10, new Pose2d( new Translation2d(12.100906, 4.1859 )     , new Rotation2d(Math.toRadians(0.0)   )));
    //     rightSideMap.put(11, new Pose2d( new Translation2d(12.457658, 3.1693908 )  , new Rotation2d(Math.toRadians(60.0)  )));

    //     //Blue Right
    //     rightSideMap.put(17, new Pose2d( new Translation2d(3.9381227, 3.1393908)   , new Rotation2d(Math.toRadians(60.0)  )));
    //     rightSideMap.put(18, new Pose2d( new Translation2d(3.5312, 3.8659 )        , new Rotation2d(Math.toRadians(0.0)   )));
    //     rightSideMap.put(19, new Pose2d( new Translation2d(3.888206, 4.8824092 )   , new Rotation2d(Math.toRadians(-60.0) )));
    //     rightSideMap.put(20, new Pose2d( new Translation2d(5.09044, 4.8824092 )    , new Rotation2d(Math.toRadians(-120.0))));
    //     rightSideMap.put(21, new Pose2d( new Translation2d(5.447446, 3.8659 )      , new Rotation2d(Math.toRadians(0.0)   )));
    //     rightSideMap.put(22, new Pose2d( new Translation2d(5.0405233, 3.1393908 ) , new Rotation2d(Math.toRadians(60.0)  )));
    // }


    /**
     * returns estimated pose if poseEstimator is not null
     */
    public Pose2d getEstimatedPose()
    {
        if(poseEstimator != null)
        {
            return estimatedPose;
        }
        else
        {
            return new Pose2d();
        }

    }

    /**
     * @param ID
     * ID of the april tag
     * @return
     * the pose of the april tag if ID is valid
     */
    public Pose2d getAprilTagPose(int ID)
    {
        if(aprilTagFieldLayout.getTagPose(ID).isEmpty())
        {
            return aprilTagFieldLayout.getTagPose(ID).get().toPose2d();
        }
        else
        {
            return null;
        }
    }

    /**
     * @param ID
     * ID of the april tag
     * @return
     * true - if tag ID is on the reef
     * false - if not
     */
    public BooleanSupplier isReefTagSupplier(double ID)
    {
        if((ID >= 6 && ID <= 11) || (ID >= 17 && ID <= 22))
        {
            return () -> true;
        }
        else
        {
            return () -> false;
        }
    }

    // public Pose2d closestBranchLocation(int aprilTagID, boolean isRight)
    // {
    //     if(isRight)
    //     {
    //         return rightSideMap.get(aprilTagID);
    //     }
    //     else
    //     {
    //         return leftSideMap.get(aprilTagID);
    //     }
    // }

    // public boolean getIsRight()
    // {
    //     return isRight;
    // }

    // public void setPlacingSideLeft()
    // {
    //     isRight = false;
    // }

    // public void setPlacingSideRight()
    // {
    //     isRight = true;
    // }

    // public Command setPlacingSideLeftCommand()
    // {
    //     return run(() -> setPlacingSideLeft());
    // }

    // public Command setPlacingSideRightCommand()
    // {
    //     return run(() -> setPlacingSideRight());
    // }

    private void periodicCameraUpdate()
    {
        if(camera != null)
        {
            if(camera.isFresh() )//&& camera.getAverageTagDistance() < MAX_TARGET_DISTANCE)
            {
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                poseEstimator.addVisionMeasurement(
                    camera.getPose2d(),
                    camera.getTimestampSeconds()
                );
            }
        }
    }

    private void periodicOdometryUpdate()
    {
        if(gyro != null && drivetrain != null)
        {
            estimatedOdometryPose = poseEstimator.update(
                gyro.getRotation2d(),
                drivetrain.getLeftLeaderDistance(),
                drivetrain.getRightLeaderDistance()
            );
        }
    }

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    
    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        periodicOdometryUpdate();

        // periodicCameraUpdate();

        estimatedPose = poseEstimator.getEstimatedPosition();
        estimatedPosePublisher.set(estimatedPose);

        // publisher.set(poseA);
        // arrayPublisher.set(new Pose2d[] {poseA, poseB} );
    }

    @Override
    public String toString()
    {
        return "Estimated pose: " + getEstimatedPose();
    }
}
