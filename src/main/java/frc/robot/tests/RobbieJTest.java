package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shifter;

public class RobbieJTest implements Test
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
    private final Shifter shifter;

    // private final Joystick joystick = new Joystick(0);
    private final CommandXboxController controller = new CommandXboxController(0);

    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here
     
    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public RobbieJTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        shifter = robotContainer.getShifter();
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
    {
        controller.x().onTrue(shifter.shiftToggleCommand());
        controller.a().onTrue(shifter.shiftLowCommand());
        controller.b().onTrue(shifter.shiftHighCommand());
    }

    /**
     * This method runs periodically (every 20ms).
     */
    public void periodic()
    {
        // System.out.println(shifter.isHighGear());

        // if(joystick.getRawButton(1))
        // {
        //     shifter.shiftHighCommand().schedule();
        //     // shifter.shiftHigh();
            
        // }

        // if(joystick.getRawButton(2))
        // {
        //     shifter.shiftLowCommand().schedule();
        //     // shifter.shiftLow();

        // }
        
        
        // if(joystick.getRawButton(3))
        // {
        //     shifter.shiftToggleCommand().schedule();
        // }

    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}