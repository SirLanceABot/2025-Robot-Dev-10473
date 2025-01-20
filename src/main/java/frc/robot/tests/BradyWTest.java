package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Roller;

public class BradyWTest implements Test
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



    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    private final RobotContainer robotContainer;
    private final Roller roller;
    private final Joystick joystick = new Joystick(0);

    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public BradyWTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        roller = robotContainer.getRoller();
        // this.exampleSubsystem = robotContainer.exampleSubsystem;

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

        

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {}

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        if(joystick.getRawButton(1))
        {
            roller.intake();
        }
        else if(joystick.getRawButton(2))
        {
            roller.eject();
        }
        else if(joystick.getRawButton(3))
        {
            roller.stop();
        }
        else if(joystick.getRawButton(4))
        {
            roller.intakeCommand().schedule();
        }
        else if(joystick.getRawButton(5))
        {
            roller.ejectCommand().schedule();
        }
        else if(joystick.getRawButton(6))
        {
            roller.stopCommand().schedule();
        }
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}