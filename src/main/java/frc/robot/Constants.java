package frc.robot;

import java.lang.invoke.MethodHandles;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public static final double MAX_BATTERY_VOLTAGE          = 12.0;
    public static final double END_OF_MATCH_BATTERY_VOLTAGE = 11.5; // This is the estimated voltage at the end of each match, used in subsystems with setVoltage()

    public static final String NETWORK_TABLE_NAME = "TeamLance";
    public static final String ADVANTAGE_SCOPE_TABLE_NAME = "ASTable";

    // These are the names of the CAN bus set on the roboRIO and CANivore
    // public static final String CANIVORE = "CANivore";
    public static final String ROBORIO  = "rio";


    public static class Camera
    {
        public static final String CAMERA = "limelight";
        public static final String BOT_POSE = "botpose_wpiblue";
        public static final String CAMERA_BOT_POSE = CAMERA + "/" + BOT_POSE;
    }

    public static class Climb
    {
        public static final int MOTOR_PORT                              = 0;
        public static final String MOTOR_CAN_BUS                        = ROBORIO;
    }

    public static class Controllers
    {
        public static final int DRIVER_CONTROLLER_PORT                  = 0;
        public static final int OPERATOR_CONTROLLER_PORT                = 1;
        public static final int SYSID_CONTROLLER_PORT                   = 2;
    }
    
    public static class Drivetrain
    {
        public static final int LEFT_LEADER_PORT                        = 3;
        public static final int LEFT_FOLLOWER_PORT                      = 4;
        public static final int RIGHT_LEADER_PORT                       = 1;
        public static final int RIGHT_FOLLOWER_PORT                     = 2;
  
        public static final String MOTOR_CAN_BUS                        = ROBORIO;
    }

    public static class Gyro
    {
        public static final int PORT                                    = 0;

        public static final String GYRO_CAN_BUS_STRING                  = ROBORIO;
    }

    public static class LEDs
    {
        public static final int LED_PORT                                = 0;
        public static final int LED_LENGTH                              = 101;
    }

    public static class Pivot
    {
        public static final int MOTOR_PORT                              = 5;
        public static final String MOTOR_CAN_BUS                        = ROBORIO;
    }

    public static class Pneumatics
    {
        public static final int PNEUMATIC_HUB_PORT                      = 1;
        
        public static final double MAX_PRESSURE                         = 120;
        public static final double MIN_PRESSURE                         = 90;

    }

    public static class Roller
    {
        public static final int MOTOR_PORT                              = 6;

        public static final String MOTOR_CAN_BUS                        = ROBORIO;

        public static final int PROXIMITY_SENSOR_RIGHT_PORT                   = 0;
        public static final int PROXIMITY_SENSOR_LEFT_PORT                    = 1;

    }

    public static class Shifter
    {
        public static final int FORWARD_CHANNEL_PORT                    = 14;
        public static final int REVERSE_CHANNEL_PORT                    = 15;        
    }

    public static class AdaptiveSlewRateLimiter
    {
        public static final double DECEL_RATE                              = 1.5;
        public static final double ACCEL_RATE                              = 1.0; 
    }

}
