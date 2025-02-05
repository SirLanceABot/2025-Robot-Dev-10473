package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Pneumatics extends Pneumatics
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
    private final PneumaticHub pneumaticHub = new pneumaticHub = new PneumaticHub(Constants.Pneumatic_HUB_PORT);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Pneumatics()
    {
        super("Pneumatics");
        System.out.println("  Constructor Started:  " + fullClassName);

        configCompressor();

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
        pneumaticHub.enableCompressorDigital();
        // One full tank can power one solenoid for 90 shifts
        
        // Disables compressor once the tank has filled once
        BooleanSupplier filledOnceSupplier = () -> (DriverStation.isTeleopEnabled()  && DriverStation.getMatchTime() < (Constants.MATCH_LENGTH - 5) && compressor.isEnabled() == false);
        
        // Disables compressor for the last 30 seconds of the match
        // BooleanSupplier pressureSupplier = () -> (DriverStation.getMatchTime() < 30.0 && DriverStation.isTeleopEnabled());

        Trigger disableTrigger = new Trigger(filledOnceSupplier);

        disableTrigger
            .onTrue(Commands.runOnce(()-> compressor.disable()));
    }

    // Better name?
    private void configAnalogSwitch
    {

    }


    private void enableCompressor()
    {
        compressor.enableCompressorDigital
    }

    private void disableCompressor()
    {
        compressor.disable
    }

    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }


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
