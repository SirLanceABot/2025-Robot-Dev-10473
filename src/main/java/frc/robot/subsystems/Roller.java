package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.SparkFlexLance;
import frc.robot.sensors.Proximity;

/**
 * @author Brady Woodard
 * @author Mason Bellinger
 */
public class Roller extends SubsystemLance
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
    private final SparkFlexLance motor = new SparkFlexLance(Constants.Roller.MOTOR_PORT, Constants.Roller.MOTOR_CAN_BUS, "Roller Motor");
    private final Proximity rightSensor = new Proximity(Constants.Roller.PROXIMITY_SENSOR_RIGHT_PORT);
    private final Proximity leftSensor = new Proximity(Constants.Roller.PROXIMITY_SENSOR_LEFT_PORT);

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Roller. 
     */
    public Roller()
    {
        super("Roller");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        motor.setupFactoryDefaults();
        motor.setupBrakeMode();
        // motor.setupVelocityConversionFactor(RPM_TO_FPS);

        motor.setSafetyEnabled(false);
        motor.setPosition(0.0);
    }

    /**
     * This sets the speed of the motors.
     * @param speed The motor speed
     */
    private void set(double speed)
    {
        motor.set(speed);
    }
    
    public void on(double speed)
    {
        motor.set(speed);
    }

    /**
     * Makes the rollers stop
     */
    private void stop()
    {
        set(0.0);
    }

    /**
     * Command to make the rollers intake
     * @return Returns intake command
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public Command intakeAlgaeCommand()
    {
        return runOnce( () -> set(0.35) )
        .withName("Intake Roller");
    }

    public Command slowRollerCommand()
    {
        return runOnce( () -> set(0.2) )
        .withName("Slow Roller");
    }

    /**
     * Command to eject coral
     * @return Returns eject coral command
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public Command ejectCoralCommand()
    {
        return runOnce( () -> set(0.15) )
        .withName("Eject Roller");
    }

    /**
     * Command to eject algae
     * @return Returns eject algae command
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public Command ejectAlgaeCommand()
    {
        return runOnce( () -> set(-0.5) )
        .withName("Eject Roller");
    }

    /**
     * Command to make the rollers stop
     * @return Returns stop command
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public Command stopCommand()
    {
        return runOnce( () -> stop() )
        .withName("Stop Roller");
    }

    /**
     * Booleansupplier to return if the proximity sensor is detecting something
     * @return Bolleansupplier for proximity sensor
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public BooleanSupplier isDetectedSupplier()
    {
        // if(leftSensor.isDetectedSupplier().getAsBoolean())
        //     return leftSensor.isDetectedSupplier();

        // return rightSensor.isDetectedSupplier();

        return () -> (leftSensor.isDetectedSupplier().getAsBoolean() || rightSensor.isDetectedSupplier().getAsBoolean());
    }   

    /**
     * Command to make the rollers intake until algae is in robot
     * @param speed Speed of the motors
     * @return Returns intake until detected command
     */
    // public Command intakeUntilDetectedCommand()
    // {
    //     return 
    //     runOnce( () -> set(0.5) )
    //     .andThen(Commands.waitUntil(sensor.isDetectedSupplier()) )
    //     .andThen(stopCommand());
    // }

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

        SmartDashboard.putNumber("Roller Velocity", motor.getVelocity());
    }

    @Override
    public String toString()
    {
        return "Roller Velocity = " + motor.getVelocity();
    }
}
