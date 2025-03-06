package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This abstract class will be extended for every subsystem on the robot. 
 * Every subsystem will automatically be added to the array list for periodic inputs and outputs.
 */
abstract class SubsystemLance extends SubsystemBase
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
     * @param subsystemName The name of the subsystem, for debugging purposes
     */
    SubsystemLance(String subsystemName)
    {
        super();

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + subsystemName);


        System.out.println("  Constructor Finished: " + fullClassName + " >> " + subsystemName);
    }
}
