package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.SparkFlexLance;

/**
 * Creates a new climb subsystem with one Kraken motor
 * @author Brady Woodard
 * @author Mason Bellinger
 */
public class Climb extends SubsystemLance
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
    private final SparkFlexLance motor = new SparkFlexLance(Constants.Climb.MOTOR_PORT, Constants.Climb.MOTOR_CAN_BUS, "Climb Motor");


    private final double FORWARD_SOFT_LIMIT = 50.0;
    private final double REVERSE_SOFT_LIMIT = 0.0;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Climb. 
     */
    public Climb()
    {
        super("Climb");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setSafetyEnabled(false);
        motor.setupBrakeMode();

        motor.setupInverted(false);
        motor.setPosition(0.0);

        motor.setupForwardSoftLimit(FORWARD_SOFT_LIMIT, true);
        motor.setupReverseSoftLimit(REVERSE_SOFT_LIMIT, true);
        motor.setupForwardHardLimitSwitch(true, true);
        motor.setupReverseHardLimitSwitch(true, true);
    }


    /**
     * Sets the motor to the speed passed to the function
     * @param speed Sets speed from -0.25 to 0.25
     */
    private void set(double speed)
    {
       motor.set(MathUtil.clamp(speed, -0.25, 0.25));
    }
    
    /**
     * Pulls the robot up
     * @param speed Speed of the motor
     */
    private void climbUp(double speed)
    {
        set(Math.abs(speed));
    }

    /**
     * Brings the robot down
     * @param speed Speed of the motor
     */
    private void climbDown(double speed)
    {
        set(-Math.abs(speed));
    }

    /**
     * Stops the climb motor
     */
    public void stop()
    {
        set(0.0);
    }

    /**
     * Pulls the robot up
     * @return Returns the climb up command
     */
    public Command climbUpCommand()
    {
        return runOnce(() -> climbUp(0.1) )
        .withName("Climb Up");
    }

    /**
     * Brings the robot down
     * @return Returns the climb down command
     */
    public Command climbDownCommand()
    {
        return runOnce(() -> climbDown(0.1) )
        .withName("Climb Down");
    }

    /**
     * Stops the climb motor
     * @return Returns the stop command
     */
    public Command stopCommand()
    {
        return runOnce(() -> stop() ).withName("Stops Climb");
    }
    
    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
    }

    @Override
    public String toString()
    {
        return "Climb position = " + motor.getPosition();
    }
}