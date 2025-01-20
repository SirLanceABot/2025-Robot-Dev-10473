package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.TargetPosition;
import frc.robot.motors.SparkFlexLance;
import frc.robot.motors.SparkMaxLance;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Pivot extends SubsystemLance
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

    private final SparkFlexLance pivotMotor = new SparkFlexLance(3, Constants.ROBORIO, "Motor 1");

    private TargetPosition targetPosition = TargetPosition.kOverride;
    private final double threshold = 0.1;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new ExampleSubsystem. 
     */
    public Pivot()
    {
        super("Pivot");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotor();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotor()
    {
        pivotMotor.setupFactoryDefaults();
        pivotMotor.setupBrakeMode();
        pivotMotor.setupInverted(true);
        pivotMotor.setPosition(0.0);
        // // pivotMotor.setSafetyEnabled(false);
        // // motor2.setupFactoryDefaults();

        // pivotMotor.setupForwardHardLimitSwitch(false, false);
        // pivotMotor.setupReverseHardLimitSwitch(false, false);

        // pivotMotor.setupPIDController(0,1,0,0);

    }

    /**
     * This sets the speed of the motor.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    public void on(double speed)
    {
        targetPosition = Constants.TargetPosition.kOverride;
        pivotMotor.set(speed);
        // motor2.set(speed);
    }

    public void hold()
    {
        targetPosition = Constants.TargetPosition.kOverride;
        pivotMotor.set(0.0);
        // motor2.set(0.0);
    }



    public Command onCommand()
    {
        return run( () -> on(0.25) );
    }

    public Command setCommand(DoubleSupplier speed)
    {
        return run( () -> on(MathUtil.clamp(speed.getAsDouble(), 0.0, 0.5)) );
    }

    public double getPosition()
    {
        return pivotMotor.getPosition();
        // return 0.0;
    }

    public void startingPosition()
    {
        targetPosition = Constants.TargetPosition.kStartingPosition;
    }

    public void grabAlgaePosition()
    {
        targetPosition = Constants.TargetPosition.kGrabAlgaePosition;
    }

    public void moveToSetPosition(Constants.TargetPosition targetPosition)
    {
        if(getPosition() > targetPosition.pivot + threshold)
        {
            on(-0.5);
        }

        else if(getPosition() > targetPosition.pivot - threshold)
        {
            on(0.5);
        }

        else
        {
            hold();
        }
    }    

    public Command onCommand(double speed)
    {
        return Commands.run(() -> on(speed), this).withName("Turn On Pivot");
    }

    public Command holdCommand()
    {
        return Commands.run(() -> hold(), this).withName("Hold Pivot");
    }

    public Command moveToSetPositionCommand(Constants.TargetPosition targetPosition)
    {
        return Commands.run(() -> moveToSetPosition(targetPosition), this).withName("Move to Set Position Pivot"); 
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
