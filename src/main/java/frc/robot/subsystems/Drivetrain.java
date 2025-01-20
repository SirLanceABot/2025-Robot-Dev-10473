package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class Drivetrain extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    private final TalonFXLance leftLeader = new TalonFXLance(Constants.Drivetrain.LEFT_LEADER_PORT, Constants.Drivetrain.LEFT_LEADER_CAN_BUS, "Left Leader");
    private final TalonFXLance leftFollower = new TalonFXLance(Constants.Drivetrain.LEFT_FOLLOWER_PORT, Constants.Drivetrain.LEFT_FOLLOWER_CAN_BUS, "Left Follower");
    private final TalonFXLance rightLeader = new TalonFXLance(Constants.Drivetrain.RIGHT_LEADER_PORT, Constants.Drivetrain.RIGHT_LEADER_CAN_BUS, "Right Leader");
    private final TalonFXLance rightFollower = new TalonFXLance(Constants.Drivetrain.RIGHT_FOLLOWER_PORT, Constants.Drivetrain.RIGHT_FOLLOWER_CAN_BUS, "Right Follower");

    private final DifferentialDrive differentialDrive = new DifferentialDrive(leftLeader, rightLeader);
    

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Drivetrain. 
     */
    public Drivetrain()
    {
        super("Example Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        configMotors();

        System.out.println("  Constructor Finished: " + fullClassName);
    }


    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    private void configMotors()
    {
        leftLeader.setupFactoryDefaults();
        leftFollower.setupFactoryDefaults();
        rightLeader.setupFactoryDefaults();
        rightFollower.setupFactoryDefaults();

        leftLeader.setupCoastMode();
        leftFollower.setupCoastMode();
        rightLeader.setupCoastMode();
        rightFollower.setupCoastMode();

        leftFollower.setupFollower(Constants.Drivetrain.LEFT_LEADER_PORT, true);
        rightFollower.setupFollower(Constants.Drivetrain.RIGHT_LEADER_PORT, true);

        leftLeader.setupInverted(true);
        rightFollower.setupInverted(true);
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

    // public void arcadeDrive(double speed, double rotation, boolean squared)
    // {
    //     drive.arcadeDrive(speed, rotation, squared);
    // }

    // public void driveTank(double leftSpeed, double rightSpeed, boolean squared)
    // {
    //     drive.tankDrive(leftSpeed, rightSpeed, squared);
    // }

    public void arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        differentialDrive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble(), squared);
    }

    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        return run( () -> arcadeDrive(speed, rotation, squared) );
    }

    public void tankDrive(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        differentialDrive.tankDrive(speed.getAsDouble(), rotation.getAsDouble(), squared);
    }

    public Command tankDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        return run( () -> tankDrive(speed, rotation, squared) );
    }

    public void stopDrive()
    {
        differentialDrive.stopMotor();
    }

    public Command stopDriveCommand()
    {
        return run( () -> stopDrive() );
    }

    public Command autonomousDriveCommand(double driveSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> 0.0, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand());
    }

    public Command autonomousTurnCommand(double turnSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> 0.0, () -> turnSpeed, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand());
    }

    @Override
    public String toString()
    {
        return "";
    }
}
