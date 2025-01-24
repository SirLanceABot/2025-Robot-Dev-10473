package frc.robot.controls;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverController extends ControllerLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final CommandXboxController xbox = new CommandXboxController(0);


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here


    public DriverController()
    {
        super("DriverController");
        System.out.println("  Constructor Started:  " + fullClassName);


        System.out.println("  Constructor Finished: " + fullClassName);
    }

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
        return "";
    }

}
