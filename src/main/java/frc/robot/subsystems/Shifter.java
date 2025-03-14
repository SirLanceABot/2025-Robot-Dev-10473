package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * This class creates a shifter subsystem
 * @author Jackson D.
 * @author Robbie J.
 */
public class Shifter extends SubsystemLance
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
    private final DoubleSolenoid solenoid = new DoubleSolenoid(Constants.Pneumatics.PNEUMATIC_HUB_PORT, PneumaticsModuleType.REVPH,
    Constants.Shifter.FORWARD_CHANNEL_PORT, Constants.Shifter.REVERSE_CHANNEL_PORT);
    private boolean isHighGear;

    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here


    /** 
     * Creates a new Shifter. 
     */
    public Shifter()
    {
        super("Shifter");
        System.out.println("  Constructor Started:  " + fullClassName);


        shiftLow();
        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here
    /**
     * Creates a method to shift to high gear.
     */
    public void shiftHigh()
    {
        solenoid.set(Value.kForward);
        isHighGear = true;
    }

    /**
     * Creates a method to go to low gear.
     */
    public void shiftLow()
    {
        solenoid.set(Value.kReverse);
        isHighGear = false;
    }

    /**
     * Returns isHigh gear boolean
     */
    public boolean isHighGear()
    {
        return isHighGear;
    }

    /**
     * Creates a method to Toggle between low and high gear on the same button.
     */
    public void shiftToggle()
    {
        if(isHighGear())
        {
            shiftLow();
        }
        else
        {
            shiftHigh();
        }
    }

    /**
     * Creates a command to run ShiftHigh method once
     */
    public Command shiftHighCommand()
    {
        return runOnce(() -> shiftHigh()).withName("Shift High");
    }

    /**
     * Creates a command to run ShiftLow method once
     */
    public Command shiftLowCommand()
    {
        return runOnce(() -> shiftLow()).withName("Shift Low");
    }

    /**
     * Creates a command to run ShiftToggle method once
     */
    public Command shiftToggleCommand()
    {
        return runOnce(() -> shiftToggle()).withName("Shift toggle");
    }

    /**
     * Creates a Boolean supplier that returns isHighGear
     */
    public BooleanSupplier isHighGearSupplier()
    {
        return () -> isHighGear();
    }

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public String toString()
    {
        return "Is High Gear:" + isHighGear;
    }
}