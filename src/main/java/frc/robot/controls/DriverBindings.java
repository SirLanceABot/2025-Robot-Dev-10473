package frc.robot.controls;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

public final class DriverBindings
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
    private static CommandXboxController controller;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here


    private DriverBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {

    }

    private static void configSuppliers()
    {
        
    }

}
