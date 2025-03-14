package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

@SuppressWarnings("unused")
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

        

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    /**
     * This method runs one time before the periodic() method.
     */
    public void init()
    {
        driveTrain.arcadeDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0), true).schedule();
    }

    /**
     * This method runs periodically (every 20ms).
     */ 
    public void periodic()
    {
        
        // if(joystick.getRawButton(1))
        // {
        //     driveTrain.prepareShiftToLowCommand().schedule();

        //     System.out.println("Prepare Shift Low");
        // }
        // else if(joystick.getRawButton(2))
        // {
        //     driveTrain.postShiftToLowCommand().schedule();

        //     System.out.println("Post Shift Low");
        // }
        // else if(joystick.getRawButton(3))
        // {
        //     driveTrain.prepareShiftToHighCommand().schedule();

        //     System.out.println("Prepare Shift High");
        // }
        // else if(joystick.getRawButton(4))
        // {
        //     driveTrain.postShiftToHighCommand().schedule();

        //     System.out.println("Post Shift High");
        // }
        // else
        // {
        //     driveTrain.arcadeDriveCommand(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), true).schedule();
        // }

        // System.out.println(driveTrain.toString());
    }
    
    /**
     * This method runs one time after the periodic() method.
     */
    public void exit()
    {} 
}