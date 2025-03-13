package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs;

@SuppressWarnings("unused")
public class NiyatiPTest implements Test
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

    private final Joystick joystick = new Joystick(0);
    private final LEDs leds;
    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public NiyatiPTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);
        
        this.robotContainer = robotContainer;
        leds = robotContainer.getLEDs();
        // this.exampleSubsystem = robotContainer.exampleSubsystem;

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

        

    // *** OVERRIDDEN METHODS ***
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
            leds.setColorGradientCommand(Color.kFuchsia, Color.kGold).schedule();
        }
        else if(joystick.getRawButton(2))
        {
            leds.offCommand().schedule();
        }
        else if(joystick.getRawButton(3))
        {
            leds.setColorRainbowCommand().schedule();
        }
        else if(joystick.getRawButton(4))
        {
            leds.setColorSolidCommand(Color.kMediumOrchid).schedule();
        }
        else if (joystick.getRawButton(5))
        {
            leds.setColorBlinkCommand(Color.kRed, Color.kFuchsia).schedule();
        }
        // else if(joystick.getRawButton(6))
        // {
        //     leds.setColorBreatheCommand(Color.kOrange, Color.kAliceBlue).schedule();
        // }
        else if(joystick.getRawButton(6))
        {
            leds.setBlindingLightCommand().schedule();
        }
        else 
        {
            System.out.println("Else");
        }

        // else if(joystick.getRawButton(5))
        // {
        //     leds.setColorCoolPatternCommand().schedule();
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     leds.setColorGradientCommand(LEDs.CustomColor.kRed).schedule();
        // }
        // else if(joystick.getRawButton(3))
        // {
        //     leds.setColorGradientCommand(LEDs.CustomColor.kRed).schedule();
        // }
        // else
        // if(joystick.getRawButton(1))
        // {
        //     leds.setColorSolidCommand(Color.kRoyalBlue).schedule();
        // }
        // {
        //     leds.stopCommand();
        // }
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}