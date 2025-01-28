package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import com.revrobotics.AnalogInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final DoubleSolenoid solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH,
    Constants.Shifter.FORWARD_CHANNEL_PORT, Constants.Shifter.REVERSE_CHANNEL_PORT);
    private static boolean isHighGear;

    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here


    /** 
     * Creates a new Shifter. 
     */
    public Shifter()
    {
        super("Shifter");
        shiftHighCommand();
        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public void shiftHigh()
    {
        solenoid.set(Value.kForward);
        isHighGear = true;
    }

    public void shiftLow()
    {
        solenoid.set(Value.kReverse);
        isHighGear = false;
    }

    public static boolean isHighGear()
    {
        return isHighGear;
    }

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

    public Command shiftHighCommand()
    {
        return runOnce(() -> shiftHigh()).withName("Shift High");
    }

    public Command shiftLowCommand()
    {
        return runOnce(() -> shiftLow()).withName("Shift Low");
    }

    public Command shiftToggleCommand()
    {
        return runOnce(() -> shiftToggle()).withName("Shift toggle");
    }

    public BooleanSupplier isHighGearSupplier()
    {
        return () -> isHighGear();
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public String toString()
    {
        return "Is High Gear:" + isHighGear;
    }
}