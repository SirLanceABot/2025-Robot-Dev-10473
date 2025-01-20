package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.motors.SparkFlexLance;

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

    private final double GEAR_RATIO = 1.0 / 5.0; // Ask Build team 
    private final double WHEEL_DIAMETER_FEET = 2.25 / 12.0; // Ask Build Team
    private final double MINUTES_TO_SECONDS = 1.0 / 60.0;
    private final double RPM_TO_FPS = GEAR_RATIO * MINUTES_TO_SECONDS * Math.PI * WHEEL_DIAMETER_FEET;
    private final double DEFAULT_VOLTAGE = 6.0;


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
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed (-1.0 to 1.0)
     */

    public void intake()
    {
        motor.setVoltage(DEFAULT_VOLTAGE);
    }

    public void eject()
    {
        motor.setVoltage(-DEFAULT_VOLTAGE);
    }

    public void stop()
    {
        motor.setVoltage(0.0);
    }

    public Command intakeCommand()
    {
        return Commands.run( () -> intake(), this).withName("Intake Roller");
    }

    public Command ejectCommand()
    {
        return Commands.run( () -> eject(), this).withName("Eject Roller");
    }

    public Command stopCommand()
    {
        return Commands.run( () -> stop(), this).withName("Stop Roller");
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
