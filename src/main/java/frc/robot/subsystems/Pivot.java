package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.motors.SparkFlexLance;

/**
 * This is the Pivot. It allows the robot to move it's arm.
 * @author Greta
 * @author Niyati
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
    public enum TargetPosition
    {
        kStartingPosition(0.0),
        kHoldAlgaePosition(2.5),
        kGrabAlgaePosition(7.75);
        // kOverride(-4237);

        public final double pivot;

        private TargetPosition(double pivot)
        {
            this.pivot = pivot;
        }
    }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private final SparkFlexLance motor = new SparkFlexLance(Constants.Pivot.MOTOR_PORT, Constants.Pivot.MOTOR_CAN_BUS, "Pivot Motor");

    // private TargetPosition targetPosition = TargetPosition.kOverride;
    private final double threshold = 0.1;
    private static boolean isWithinThreshold;

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
        motor.setupFactoryDefaults();
        motor.setupBrakeMode();
        motor.setupInverted(true);
        motor.setPosition(0.0);
        motor.setSafetyEnabled(false);
        // // motor2.setupFactoryDefaults();

        // pivotMotor.setupForwardHardLimitSwitch(false, false);
        // pivotMotor.setupReverseHardLimitSwitch(false, false);

        motor.setupForwardSoftLimit(8.75, true);
        // motor.setupReverseSoftLimit(0.0, true);

        motor.setupPIDController(0, 0.025, 0, 0);

        motor.setPosition(0.0);
    }

    private void set(double speed)
    {
        motor.set( MathUtil.clamp(speed, -0.3, 0.3));
    }

    /**
     * This sets the speed of the motor.
     * @param speed The motor speed (-1.0 to 1.0)
     */
    public void on(double speed)
    {
        // targetPosition = Constants.TargetPosition.kOverride;
        set(speed);
        // motor2.set(speed);
    }

    private void hold()
    {
        // targetPosition = Constants.TargetPosition.kOverride;
        set(0.0);
        // motor2.set(0.0);
    }

    // public Command onCommand()
    // {
    //     return run( () -> set(0.25) );
    // }

    // public Command setCommand(DoubleSupplier speed)
    // {
    //     return run( () -> set(speed.getAsDouble()) );
    // }

    public double getPosition()
    {
        return motor.getPosition();
        // return 0.0;
    }

    /**
     * Checks if the pivot is within the threshold of the targetPosition
     * @param targetPosition
     * @return isWithinThreshold
     */
    public boolean isWithinThreshold(TargetPosition targetPosition)
    {
        if((getPosition() == targetPosition.pivot + threshold) || (getPosition() == targetPosition.pivot - threshold))
            isWithinThreshold = true;
        else
            isWithinThreshold = false;
        return isWithinThreshold;
    }

    // public boolean within_threshold(TargetPosition targetPosition)
    // {
    //     if((getPosition() == targetPosition.pivot + threshold) || (getPosition() == targetPosition.pivot - threshold))
    //         return true;
    //     else
    //         return false;
    // }

    // public void startingPosition()
    // {
    //     targetPosition = Constants.TargetPosition.kStartingPosition;
    // }

    // public void grabAlgaePosition()
    // {
    //     targetPosition = Constants.TargetPosition.kGrabAlgaePosition;
    // }

    // public void moveToSetPosition(Constants.TargetPosition targetPosition)
    // {
    //     if(getPosition() > targetPosition.pivot + threshold)
    //     {
    //         on(-0.5);
    //     }

    //     else if(getPosition() > targetPosition.pivot - threshold)
    //     {
    //         on(0.5);
    //     }

    //     else
    //     {
    //         hold();
    //     }
    // }    

    public BooleanSupplier isWithinThresholdSupplier(TargetPosition targetPosition)
    {
        return () -> isWithinThreshold(targetPosition);
    }

    public Command onCommand(double speed)
    {
        return run(() -> set(speed)).withName("Turn On Pivot");
    }

    public Command holdCommand()
    {
        return run(() -> hold()).withName("Hold Pivot");
    }

    public Command moveToSetPositionCommand(TargetPosition targetPosition)
    {
        return run(() -> motor.setControlPosition(targetPosition.pivot));
        // return Commands.run(() -> moveToSetPosition(targetPosition), this).withName("Move to Set Position Pivot"); 
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

        SmartDashboard.putNumber("Pivot Position", getPosition());
    }

    @Override
    public String toString()
    {
        return "";
    }
}
