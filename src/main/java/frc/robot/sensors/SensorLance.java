package frc.robot.sensors;

import java.lang.invoke.MethodHandles;

import frc.robot.PeriodicTask;

/**
 * This abstract class will be extended for every sensor on the robot. 
 * Every sensor will automatically be added to the array list for periodic inputs, outputs, and tasks.
 */
abstract class SensorLance implements PeriodicTask
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Registers the sensor for PeriodIO and PeriodTask.
     * @param sensorName The name of this sensor, for debugging purposes
     */
    SensorLance(String sensorName)
    {
        super();

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + sensorName);

        // Register this sensor in the array list to run periodic tasks
        registerPeriodicTask();

        System.out.println("  Constructor Finished: " + fullClassName + " >> " + sensorName);
    }


    // *** ABSTRACT METHODS ***
    // These methods must be defined in any subclass that extends this class
    public abstract void periodic();
}
