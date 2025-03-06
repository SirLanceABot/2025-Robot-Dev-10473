package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.motors.TalonFXLance;

@SuppressWarnings("unused")
public class MasonBTest implements Test
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
    private final TalonFXLance motor = new TalonFXLance(2, "rio", "kraken motor");
    private final Joystick joystick = new Joystick(0);
    // private final DigitalInput forwardHardLimit = new DigitalInput(0);
    // private final DigitalInput reverseHardLimit = new DigitalInput(1);
    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public MasonBTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        motor.setupFactoryDefaults();
        motor.setupPIDController(0, 1, 0, 0);
        motor.setupForwardHardLimitSwitch(true, true, 0);
        motor.setupReverseHardLimitSwitch(true, true, 1);
        motor.setupBrakeMode();
        // motor.setupReverseHardLimitSwitch(true, true);
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
            // motor.setControlPosition(100, forwardHardLimit.get());
            motor.setControlPosition(150);
        }
        else if(joystick.getRawButton(2))
        {
            motor.setControlPosition(-150);
        }
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}