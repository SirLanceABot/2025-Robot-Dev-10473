package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.sensors.Camera;
import frc.robot.sensors.GyroLance;

/**
 * @author Robbie Frank
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
    private final Camera camera;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final NetworkTable ASTable;
    private final DoubleArrayEntry poseEstimatorEntry;

    private boolean isRight;

    private Pose2d estimatedPose = new Pose2d();

    Pose2d poseA = new Pose2d();
    Pose2d poseB = new Pose2d();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

    private final double[] defaultValues = {0.0, 0.0, 0.0};
    private final double MAX_TARGET_DISTANCE = 5.0; // meters

    private final List<Pose2d> aprilTagLocations = new ArrayList<Pose2d>(){{
        new Pose2d(new Translation2d(13.474446, 3.306318) , new Rotation2d(Math.toRadians(120.0)  ));  // check angle values
        new Pose2d(new Translation2d(13.890498, 4.0259)   , new Rotation2d(Math.toRadians(180.0)  ));
        new Pose2d(new Translation2d(13.474446, 4.745482) , new Rotation2d(Math.toRadians(-120.0) ));
        new Pose2d(new Translation2d(12.643358, 4.745482) , new Rotation2d(Math.toRadians(-60.0)  ));
        new Pose2d(new Translation2d(12.227306, 4.0259)   , new Rotation2d(Math.toRadians(0.0)    ));
        new Pose2d(new Translation2d(12.643358, 3.306318) , new Rotation2d(Math.toRadians(60.0)   ));

        new Pose2d(new Translation2d(4.073906, 3.30618)   , new Rotation2d(Math.toRadians(60.0)   ));
        new Pose2d(new Translation2d(3.6576, 4.0259)      , new Rotation2d(Math.toRadians(0.0)    ));
        new Pose2d(new Translation2d(4.073906, 4.745482)  , new Rotation2d(Math.toRadians(-60.0)  ));
        new Pose2d(new Translation2d(4.90474, 4.745482)   , new Rotation2d(Math.toRadians(-120.0) ));
        new Pose2d(new Translation2d(5.321046, 4.0259)    , new Rotation2d(Math.toRadians(180.0)  ));
        new Pose2d(new Translation2d(4.90474, 3.306318)   , new Rotation2d(Math.toRadians(120.0)  ));
    
        // order should stay the same
    }};

    private final HashMap<Integer, Pose2d> rightSideMap = new HashMap<Integer, Pose2d>();
    private final HashMap<Integer, Pose2d> leftSideMap = new HashMap<Integer, Pose2d>();

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new PoseEstimator. 
     * 
     * tracks the robot's pose using data from the gyro, drivetrain, and the camera
     */
    public PoseEstimatorLance(GyroLance gyro, Drivetrain drivetrain, Camera camera)
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
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
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

    public void resetPoseEstimator(Pose2d pose)
    {
        poseEstimator.resetPosition(
            gyro.getRotation2d(),
            drivetrain.getLeftLeaderDistance(),
            drivetrain.getRightLeaderDistance(),
            pose
        );
    }

    private void periodicCameraUpdate()
    {
        if(camera != null)
        {
            if(camera.isTargetFound() && camera.getAverageTagDistance() < MAX_TARGET_DISTANCE)
            {
                LimelightHelpers.PoseEstimate limelightMeasurement = camera.getPoseEstimate();
                if(limelightMeasurement.tagCount >= 2 )     // only trusting if multiple tags are seen
                {
                    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                    poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds
                    );
                }
            }
        }
    }

    public void fillMaps()
    {
        //Red Left
        leftSideMap.put( 6,  new Pose2d( new Translation2d(13.6102433, 3.1393908 ) , new Rotation2d(Math.toRadians(120.0)  )));
        leftSideMap.put( 7,  new Pose2d( new Translation2d(14.016898, 3.8659 )     , new Rotation2d(Math.toRadians(180.0)  )));
        leftSideMap.put( 8,  new Pose2d( new Translation2d(13.660146, 4.8824092 )  , new Rotation2d(Math.toRadians(-120.0) )));
        leftSideMap.put( 9,  new Pose2d( new Translation2d(12.457658, 4.8827472 )  , new Rotation2d(Math.toRadians(-60.0)  )));
        leftSideMap.put( 10, new Pose2d( new Translation2d(12.100906, 3.8659 )     , new Rotation2d(Math.toRadians(0.0)    )));
        leftSideMap.put( 11, new Pose2d( new Translation2d(12.5075747, 3.1393908 ) , new Rotation2d(Math.toRadians(60.0)   )));

        //Blue Left
        leftSideMap.put( 17, new Pose2d( new Translation2d(3.888206, 3.1693908 )   , new Rotation2d(Math.toRadians(60.0)   )));
        leftSideMap.put( 18, new Pose2d( new Translation2d(3.5312, 4.1859 )        , new Rotation2d(Math.toRadians(0.0)    )));
        leftSideMap.put( 19, new Pose2d( new Translation2d(3.9381227, 4.9124092 )  , new Rotation2d(Math.toRadians(-60.0)  )));
        leftSideMap.put( 20, new Pose2d( new Translation2d(5.0405233, 4.9124092 )  , new Rotation2d(Math.toRadians(-120.0) )));
        leftSideMap.put( 21, new Pose2d( new Translation2d(5.447446, 4.1859 )      , new Rotation2d(Math.toRadians(180.0)  )));
        leftSideMap.put( 22, new Pose2d( new Translation2d(5.06044, 3.1693908 )    , new Rotation2d(Math.toRadians(120.0)  )));

        //Red Right
        rightSideMap.put(6,  new Pose2d( new Translation2d(13.660146, 3.139308)    , new Rotation2d(Math.toRadians(120.0) )));
        rightSideMap.put(7,  new Pose2d( new Translation2d(14.016898, 4.1859 )     , new Rotation2d(Math.toRadians(180.0) )));
        rightSideMap.put(8,  new Pose2d( new Translation2d(13.6102293, 4.9124092 ) , new Rotation2d(Math.toRadians(-120.0))));
        rightSideMap.put(9,  new Pose2d( new Translation2d(12.5075747, 4.9124092 ) , new Rotation2d(Math.toRadians(-60.0) )));
        rightSideMap.put(10, new Pose2d( new Translation2d(12.100906, 4.1859 )     , new Rotation2d(Math.toRadians(0.0)   )));
        rightSideMap.put(11, new Pose2d( new Translation2d(12.457658, 3.1693908 )  , new Rotation2d(Math.toRadians(60.0)  )));

        //Blue Right
        rightSideMap.put(17, new Pose2d( new Translation2d(3.9381227, 3.1393908)   , new Rotation2d(Math.toRadians(60.0)  )));
        rightSideMap.put(18, new Pose2d( new Translation2d(3.5312, 3.8659 )        , new Rotation2d(Math.toRadians(0.0)   )));
        rightSideMap.put(19, new Pose2d( new Translation2d(3.888206, 4.8824092 )   , new Rotation2d(Math.toRadians(-60.0) )));
        rightSideMap.put(20, new Pose2d( new Translation2d(5.09044, 4.8824092 )    , new Rotation2d(Math.toRadians(-120.0))));
        rightSideMap.put(21, new Pose2d( new Translation2d(5.447446, 3.8659 )      , new Rotation2d(Math.toRadians(0.0)   )));
        rightSideMap.put(22, new Pose2d( new Translation2d(5.0405233, 3.1393908 ) , new Rotation2d(Math.toRadians(60.0)  )));
    }


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

    public Pose2d getAprilTagPose(int index)
    {
        return aprilTagLocations.get(index);
    }

    public BooleanSupplier isReefTagSupplier(double tagID)
    {
        if((tagID >= 6 && tagID <= 11) || (tagID >= 17 && tagID <= 22))
        {
            return () -> true;
        }
        else
        {
            return () -> false;
        }
    }

    public Pose2d closestBranchLocation(int aprilTagID, boolean isRight)
    {
        if(isRight)
        {
            return rightSideMap.get(aprilTagID);
        }
        else
        {
            return leftSideMap.get(aprilTagID);
        }
    }

    public boolean getIsRight()
    {
        return isRight;
    }

    public void setPlacingSideLeft()
    {
        isRight = false;
    }

    public void setPlacingSideRight()
    {
        isRight = true;
    }

    public Command setPlacingSideLeftCommand()
    {
        return run(() -> setPlacingSideLeft());
    }

    public Command setPlacingSideRightCommand()
    {
        return run(() -> setPlacingSideRight());
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        estimatedPose = poseEstimator.update(
            gyro.getRotation2d(),
            drivetrain.getLeftLeaderDistance(),
            drivetrain.getRightLeaderDistance()
        );

        publisher.set(poseA);
        arrayPublisher.set(new Pose2d[] {poseA, poseB} );
    }

    @Override
    public String toString()
    {
        return "Estimated pose: " + getEstimatedPose();
    }
}
