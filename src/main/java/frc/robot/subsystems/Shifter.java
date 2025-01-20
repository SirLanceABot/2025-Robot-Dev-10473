
package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

/**
 * Drivetrain Shifter subsystem.
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
    
    public enum shiftPosition
    {
        kHigh(Value.kForward),
        kLow(Value.kReverse);

        public final Value shifterValue;

        private shiftPosition(Value shifterValue)
        {
            this.shifterValue = shifterValue;
        }
    }

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here


    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private boolean isHighGear;
    
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
    Constants.Shifter.FORWARD_CHANNEL_PORT, Constants.Shifter.REVERSE_CHANNEL_PORT);

    /** 
     * Creates a new Shifter. 
     */
    public Shifter()
    {
        super("Shifter");
        System.out.println("  Constructor Started:  " + fullClassName);
        setDefaultCommand(shiftHighCommand());
        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    public void shiftHigh()
    {
        solenoid.set(shiftPosition.kHigh.shifterValue);
        isHighGear = true;
    }

    public void shiftLow()
    {
        solenoid.set(shiftPosition.kLow.shifterValue);
        isHighGear = false;
    }

    public boolean isHighGear()
    {
        return isHighGear;
    }

    public void shiftToggle()
    {
        if(isHighGear())
        {
            shiftLowCommand().schedule();
        }
        else
        {
            shiftHighCommand().schedule();
        }
    }

    public Command shiftHighCommand()
    {
        return Commands.runOnce(() -> shiftHigh(), this).withName("Shift High");
    }

    public Command shiftLowCommand()
    {
        return Commands.runOnce(() -> shiftLow(), this).withName("Shift Low");
    }

    public Command shiftToggleCommand()
    {
        return Commands.runOnce(() -> shiftToggle(), this).withName("Shift toggle");
    }

    public BooleanSupplier isHighGearSupplier()
    {
        return () -> isHighGear();
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
        return "Is High Gear:" + isHighGear;
    }
}