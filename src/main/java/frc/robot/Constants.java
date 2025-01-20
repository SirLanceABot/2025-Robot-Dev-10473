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

    public static final double MAX_BATTERY_VOLTAGE          = 12.0;
    public static final double END_OF_MATCH_BATTERY_VOLTAGE = 11.5; // This is the estimated voltage at the end of each match, used in subsystems with setVoltage()
    
    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public static final String NETWORK_TABLE_NAME = "TeamLHS";
    public static final String ADVANTAGE_SCOPE_TABLE_NAME = "ASTable";

    // These are the names of the CAN bus set on the roboRIO and CANivore
    public static final String CANIVORE = "CANivore";
    public static final String ROBORIO  = "rio";

    public static class Shifter
    {
        public static final int FORWARD_CHANNEL_PORT                    = 0;
        public static final int REVERSE_CHANNEL_PORT                    = 0;        
    }

    public static class Drivetrain
    {
        public static final int LEFT_LEADER_PORT                        = 0;
        public static final int LEFT_FOLLOWER_PORT                      = 0;
        public static final int RIGHT_LEADER_PORT                       = 0;
        public static final int RIGHT_FOLLOWER_PORT                     = 0;

        public static final String LEFT_LEADER_CAN_BUS                  = CANIVORE;
        public static final String LEFT_FOLLOWER_CAN_BUS                = CANIVORE;
        public static final String RIGHT_LEADER_CAN_BUS                 = CANIVORE;
        public static final String RIGHT_FOLLOWER_CAN_BUS               = CANIVORE;
    }

    public static class Pivot
    {
        public static final int MOTOR_PORT                              = 0;
        public static final String MOTOR_CAN_BUS                        = CANIVORE;

        public static final double STARTING_POSITION                    = 0.0;
        public static final double GRAB_ALGAE_POSITION                  = 0.0;
    }

    public static class Roller
    {
        public static final int MOTOR_PORT                              = 0;

        public static final String MOTOR_CAN_BUS                        = ROBORIO;
    }

    public enum TargetPosition
    {
        kStartingPosition(Constants.Pivot.STARTING_POSITION),
        kGrabAlgaePosition(Constants.Pivot.GRAB_ALGAE_POSITION),
        kOverride(-4237);

        public final double pivot;

        private TargetPosition(double pivot)
        {
            this.pivot = pivot;
        }
    }
}
