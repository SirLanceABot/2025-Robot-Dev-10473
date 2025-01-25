
/**
 * @author
 * Mason Bellinger
 * Brady Woodard
 */
package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.motors.SparkFlexLance;
import frc.robot.sensors.Proximity;

/**
 * This is an example of what a subsystem should look like.
 */
public class Roller extends SubsystemLance
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
    private final SparkFlexLance motor = new SparkFlexLance(Constants.Roller.MOTOR_PORT, Constants.Roller.MOTOR_CAN_BUS, "Roller Motor");
    private final Proximity sensor = new Proximity(0);

    private final double GEAR_RATIO = 1.0 / 5.0; // Ask Build team 
    private final double WHEEL_DIAMETER_FEET = 2.25 / 12.0; // Ask Build Team
    private final double MINUTES_TO_SECONDS = 1.0 / 60.0;
    private final double RPM_TO_FPS = GEAR_RATIO * MINUTES_TO_SECONDS * Math.PI * WHEEL_DIAMETER_FEET;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Roller. 
     */
    public Roller()
    {
        super("Roller");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setupVelocityConversionFactor(RPM_TO_FPS);

        motor.setSafetyEnabled(false);
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed
     */
    private void set(double speed)
    {
        motor.set(speed);
    }
    
    /**
     * Makes the rollers intake
     * @param speed The motor speed
     */
    public void intake(double speed)
    {
        set(speed);
    }

    /**
     * Makes the rollers eject
     * @param speed The motor speed
     */
    public void eject(double speed)
    {
        set(-speed);
    }

    /**
     * Makes the rollers stop
     */
    public void stop()
    {
        set(0.0);
    }

    /**
     * Command to make the rollers intake
     * @param speed Speed of the motors
     * @return Returns intake command
     */
    public Command intakeCommand(double speed)
    {
        return runOnce( () -> intake(speed) )
        // .withTimeout(time)
        .withName("Intake Roller");
    }

    /**
     * Command to make the rollers eject
     * @param speed Speed of the motors
     * @param time  How long the motors run (seconds)
     * @return Returns eject command
     */
    public Command ejectCommand(double speed, double time)
    {
        return runOnce( () -> eject(speed) )
        .withTimeout(time)
        .withName("Eject Roller");
    }

    /**
     * Command to make the rollers stop
     * @return Returns stop command
     */
    public Command stopCommand()
    {
        return runOnce( () -> stop() ).withName("Stop Roller");
    }

    /**
     * Command to make the rollers intake until algae is in robot
     * @param speed Speed of the motors
     * @return Returns intake until detected command
     */
    public Command intakeUntilDetectedCommand(double speed)
    {
        return 
        runOnce( () -> intake(speed) )
        .andThen(Commands.waitUntil(sensor.isDetectedSupplier()) )
        .andThen(stopCommand());
    }


    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }


    // *** OVERRIDEN METHODS ***
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
        return "Roller Velocity = " + motor.getVelocity();
    }
}
