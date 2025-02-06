package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.motors.TalonFXLance;
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

    private final double[] defaultValues = {0.0, 0.0, 0.0};
    private final double MAX_TARGET_DISTANCE = 5.0; // meters

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
            if(camera.isTargetFound() && camera.getAverageDistanceFromTarget() < MAX_TARGET_DISTANCE)
            {
                LimelightHelpers.PoseEstimate limelightMeasurement =
                LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
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


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.

        poseEstimator.update(
            gyro.getRotation2d(),
            drivetrain.getLeftLeaderDistance(),
            drivetrain.getRightLeaderDistance()
        );
    }

    @Override
    public String toString()
    {
        return "";
    }
}
