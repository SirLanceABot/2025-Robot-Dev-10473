package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class RobbieFTest implements Test
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
    private final Drivetrain driveTrain;

    private final Joystick joystick = new Joystick(0);
    // private final ExampleSubsystem exampleSubsystem;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Use this class to test your code using Test mode
     * @param robotContainer The container of all robot components
     */
    public RobbieFTest(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        driveTrain = robotContainer.getDrivetrain();
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
        // driveTrain.arcadeDriveCommand(() -> joystick.getRawAxis(1) / 2, () -> joystick.getRawAxis(0) / 2, true).schedule();

        if(joystick.getRawButton(1))
        {
            System.out.println("A Button");
            driveTrain.autonomousDriveCommand(0.1, 5.0).schedule();
            // driveTrain.arcadeDriveCommand(() -> 0.25, () -> 0.0, false).schedule();
        }
        // else
        // {
        //     driveTrain.stopDriveCommand().schedule();
        // }
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}