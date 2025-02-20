package frc.robot.sensors;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;

public class Proximity extends SensorLance
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
    private final DigitalInput sensor;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Proximity. 
     */
    public Proximity(int port)
    {   
        super("Proximity");
        System.out.println("  Constructor Started:  " + fullClassName);

        sensor = new DigitalInput(port);

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here


    
    /**
     * Returns the value of the sensor
     * @return The value of the sensor(true/false)
     */
    public boolean getSensorValue()
    {
        return !sensor.get();
    }

    public BooleanSupplier isDetectedSupplier()
    {
        return () -> getSensorValue();
    }

    public BooleanSupplier isNotDetectedSupplier()
    {
        return () -> !getSensorValue();
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {

    }

    @Override
    public String toString()
    {
        return "Proximity Value = " + getSensorValue();
    }
}