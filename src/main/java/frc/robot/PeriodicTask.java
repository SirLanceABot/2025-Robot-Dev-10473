package frc.robot;

import java.util.ArrayList;

/**
 * This interface is implemented for every system on the robot (excluding SubsystemBase) that runs periodic tasks.
 * Every class must call the <b>registerPeriodicTask()</b> method to add the system to the array list for periodic inputs and outputs.
 */
public interface PeriodicTask
{
    // *** STATIC CONSTANTS ***
    // Put all constants here - automatically "public final static"
    public final static ArrayList<PeriodicTask> allPeriodicTasks = new ArrayList<PeriodicTask>();


    // *** STATIC METHODS ***
    // Put all static methods here - automatically "public"

    /**
     * Static method to periodically update all of the systems in the array list.
     * Call this method from the robotPeriodic() method in the Robot class.
     */
    public static void runAllPeriodicTasks()
    {
        for(PeriodicTask periodicTask : allPeriodicTasks)
            periodicTask.periodic();
    }


    // *** DEFAULT METHODS ***
    // Put all default methods methods here - automatically "public"

    /**
     * Default method to register periodic inputs and outputs
     */
    public default void registerPeriodicTask()
    {
        allPeriodicTasks.add(this);
    }


    // *** ABSTRACT METHODS ***
    // Put all abstract methods here - automatically "public abstract"
    // These methods must be defined in any subclass that implements this interface

    public abstract void periodic();
}
