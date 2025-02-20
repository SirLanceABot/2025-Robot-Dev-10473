package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class GyroLance extends SensorLance
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


    public enum ResetState
    {
        kStart, kTry, kDone;
    }


    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
       
    // Inputs
    private double yaw;
    private double pitch;
    private double roll;
    private Rotation2d rotation2d;

    // Outputs
    private DoubleEntry yawEntry;
    private DoubleEntry xAxisEntry;
    private DoubleEntry yAxisEntry;


    public static final double RESET_DELAY = 0.1;

    
    private final Pigeon2 gyro = new Pigeon2(Constants.Gyro.PORT, Constants.Gyro.GYRO_CAN_BUS_STRING);
    private ResetState resetState = ResetState.kDone;
    private Timer timer = new Timer();

    private NetworkTable ASTable;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Gyro. 
     */
    public GyroLance()
    {
        super("Gyro");
        System.out.println("  Constructor Started:  " + fullClassName);

        ASTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        yawEntry = ASTable.getDoubleTopic("GyroYaw").getEntry(0.0);
        xAxisEntry = ASTable.getDoubleTopic("accelXAxis").getEntry(0.0);
        yAxisEntry = ASTable.getDoubleTopic("accelYAxis").getEntry(0.0);

        configGyro();
        rotation2d = gyro.getRotation2d();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configGyro()
    {
        Pigeon2Configuration configs = new Pigeon2Configuration();
        MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
        Pigeon2FeaturesConfigs featuresConfigs = new Pigeon2FeaturesConfigs();
        GyroTrimConfigs gyroTrimConfigs = new GyroTrimConfigs();

        StatusCode statusCode;
        int count = 0;

        do
        {
            // Reset to factory defaults
            statusCode = gyro.getConfigurator().apply(configs);
            count++;
        }
        while(!statusCode.isOK() && count < 5);

        mountPoseConfigs.withMountPoseRoll(0.0);
        mountPoseConfigs.withMountPosePitch(0.0);
        mountPoseConfigs.withMountPoseYaw(0.0);
        configs.withMountPose(mountPoseConfigs);

        featuresConfigs.withDisableNoMotionCalibration(false);
        featuresConfigs.withDisableTemperatureCompensation(false);
        featuresConfigs.withEnableCompass(false);
        configs.withPigeon2Features(featuresConfigs);

        gyroTrimConfigs.withGyroScalarX(0.0);
        gyroTrimConfigs.withGyroScalarY(0.0);
        gyroTrimConfigs.withGyroScalarZ(0.0);

        count = 0;
        do
        {
            // Apply gyro configurations
            statusCode = gyro.getConfigurator().apply(configs);
            count++;
        }
        while(!statusCode.isOK() && count < 5);

        gyro.reset();
        Timer.delay(0.5);
    }

    /**
     * Goes through reset process to eventually get yaw to zero degrees
     */
    public void reset()
    {
        resetState = ResetState.kStart;
    }

    /**
     * Wrapper for Pigeon2 setYaw() method
     * @param newValue Value to set to. Units are in deg.
     */
    public void setYaw(double newValue)
    {
        gyro.setYaw(newValue);
    }

    public double getRoll()
    {
        return roll; // x-axis
    }

    public double getPitch()
    {
        return pitch; // y-axis
    }

    public double getYaw()
    {
        return yaw; // z-axis
    }

    public Rotation2d getRotation2d()
    {
        return rotation2d;
    }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        switch(resetState)
        {
            case kStart:
                gyro.reset();
                timer.reset();
                timer.start();
                gyro.setYaw(0.0);
                resetState = ResetState.kTry;
                break;
            case kTry:
                if(timer.hasElapsed(RESET_DELAY))
                    resetState = ResetState.kDone;
                break;
            case kDone:
                break;
        }

        if(resetState == ResetState.kDone)
        {
            yaw = gyro.getYaw().getValueAsDouble();
            pitch = gyro.getPitch().getValueAsDouble();
            roll = gyro.getRoll().getValueAsDouble();

            rotation2d = gyro.getRotation2d();
        }
        
        yawEntry.set(yaw);
        xAxisEntry.set(gyro.getAccelerationX().getValueAsDouble());
        yAxisEntry.set(gyro.getAccelerationY().getValueAsDouble());
    }


    @Override
    public String toString()
    {
        return String.format("Gyro %f \n", yaw);
    }
}