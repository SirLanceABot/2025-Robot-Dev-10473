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

    /**
     * @param speed
     * sets drivetrain speed
     * @param rotation
     * sets drivetrain rotation speed
     * @param squared
     * boolean for squaring the outputs to limit drivetrain sensitivity 
     */
    public void arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        differentialDrive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble(), squared);
    }

    /**
     * @param speed
     * sets motor speed of arcadeDrive method
     * @param rotation
     * sets rotation speed of arcadeDrive method
     * @param squared
     * determines if outputs are squared
     * @return
     * Command returning the arcadeDrive method
     */
    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean squared)
    {
        return run( () -> arcadeDrive(speed, rotation, squared) );
    }

    /**
     * @param driveSpeed
     * sets drivetrain speed
     * @param rotationSpeed
     * sets drivetrain rotation speed
     * @param squared
     * boolean for squaring the outputs to limit drivetrain sensitivity 
     */
    public void tankDrive(DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed, boolean squared)
    {
        differentialDrive.tankDrive(driveSpeed.getAsDouble(), rotationSpeed.getAsDouble(), squared);
    }

    /**
     * @param driveSpeed
     * sets motor speed of arcadeDrive method
     * @param rotationSpeed
     * sets rotation speed of arcadeDrive method
     * @param squared
     * determines if outputs are squared
     * @return
     * Command returning the tankDrive method
     */
    public Command tankDriveCommand(DoubleSupplier driveSpeed, DoubleSupplier rotationSpeed, boolean squared)
    {
        return run( () -> tankDrive(driveSpeed, rotationSpeed, squared) );
    }

    /**
     * stops the drivetrain motors
     */
    public void stopDrive()
    {
        differentialDrive.stopMotor();
    }

    /**
     * @return
     * returns the stopDrive method as a Command
     */
    public Command stopDriveCommand()
    {
        return run( () -> stopDrive() ).withName("Stop Drive");
    }

    /**
     * @param driveSpeed
     * sets speed of arcadeDriveCommand
     * @param driveTime
     * time for which the Command will run
     * @return
     */
    public Command autonomousDriveCommand(double driveSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> 0.0, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand())
            .withName("Autonomous Drive Command");
    }

    /**
     * 
     * @param rotationSpeed
     * @param driveTime
     * @return
     */
    public Command autonomousTurnCommand(double rotationSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> 0.0, () -> rotationSpeed, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand())
            .withName("Autonomous Turn Command");
    }

    /**
     * 
     * @param driveSpeed
     * @param rotationSpeed
     * @param driveTime
     * @return
     */
    public Command autonomousTurnAndDriveCommand(double driveSpeed, double rotationSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> rotationSpeed, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand())
            .withName("Autonomous Turn And Drive Command");
    }

    @Override
    public String toString()
    {
        return "Left Leader Motor Velo = " + leftLeader.getVelocity() + "Left Follower Motor Velo = " + leftFollower.getVelocity() + "Right Leader Motor Velo = " + rightLeader.getVelocity() + "Right Follower Motor Velo = " + rightFollower.getVelocity();
    }
}
