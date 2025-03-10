package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * This is an example of what a subsystem should look like.
 */
public class Pneumatics extends SubsystemLance
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

    private final Compressor compressor=  new Compressor(PneumaticsModuleType.REVPH);
    private final PneumaticHub pneumaticHub = new PneumaticHub(Constants.Pneumatics.PNEUMATIC_HUB_PORT);
    // private final AnalogInput analogPressure = new AnalogInput(0);


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Pneumatics subsystem. 
     */
    public Pneumatics()
    {
        super("Pneumatics");
        System.out.println("  Constructor Started:  " + fullClassName);

        configCompressor();
        configAnalogSwitch(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

  /**
     * Sets up the compressor and sets it to stop once the tank has filled once
     * @author Jackson D
     * @author Robbie J 
     */
    private void configCompressor()
    {
        // One full tank can power one solenoid for 90 shifts
        
        // Disables compressor once the tank has filled once
        // BooleanSupplier filledOnceSupplier = () -> (DriverStation.isTeleopEnabled()  && DriverStation.getMatchTime() < (Constants.MATCH_LENGTH - 5) && compressor.isEnabled() == false);
        
        // Disables compressor for the last 30 seconds of the match
        BooleanSupplier pressureSupplier = () -> (DriverStation.getMatchTime() < 30.0 && DriverStation.isTeleopEnabled() && pneumaticHub.getPressure(0) > 100);

        Trigger disableTrigger = new Trigger(pressureSupplier);

        disableTrigger
            .onTrue(Commands.runOnce(()-> compressor.disable()));
    }

    /**
     * 
     * @param min Sets the minumum pressure, the compressor turns on below this value
     * @param max Sets the maximum pressure, the compressor will turn off at this value
     */
    public void configAnalogSwitch(double min, double max)
    {
        pneumaticHub.enableCompressorAnalog(min, max); 
    }

    /**
     * Enables the compressor
     */    
    public void enableCompressor()
    {
        compressor.enableAnalog(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);
    }

    /**
     * Disables the compressor
     */
    public void disableCompressor()
    {
        compressor.disable();
    }

    /**
     * @return Current pressure of the compressor in PSI
     */
    public double getPressure()
    {
        return pneumaticHub.getPressure(0);
    }

    /**
     * @return Current pressure of the compressor in PSI
     */
    public DoubleSupplier getPressureSupplier()
    {
        return () -> getPressure();
    }

    /**
     * Disables the compressor
     */
    public Command disableCompressorCommand()
    {
        return runOnce(() -> disableCompressor()).withName("Disable compressor");
    }

    /**
     * Enables the compressor
     */
    public Command enableCompressorCommand()
    {
        return runOnce(() -> enableCompressor()).withName("Enable compressor");
    }
    // *** OVERRIDDEN METHODS ***
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
