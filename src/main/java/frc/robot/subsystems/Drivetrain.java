package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;

/**
 * @author Robbie Frank 
 * @author Aditya Yadav
 * 
 * creates new drivetrain
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

    private final double MAXLOWGEARSPEED = 0.53125;

    private final double FIRSTSTAGEGEARRATIO = 12.0 / 60.0;
    private final double SECONDSTAGEGEARRATIO = 24.0 / 32.0;
    private final double HIGHGEARRATIO = FIRSTSTAGEGEARRATIO * SECONDSTAGEGEARRATIO * (44.0 / 22.0);    // 0.3
    private final double LOWGEARRATIO = FIRSTSTAGEGEARRATIO * SECONDSTAGEGEARRATIO * (34.0 / 32.0);     // 0.159375

    // divisor is the number to divide the high gear speed ouputs by to match the max low gear speed
    // to allow smooth shifting
    private double divisor = 1.0;
    // actual value of high gear to low gear is 1.882352941, rounded to 1.9 for tolerance
    

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

        leftLeader.setupBrakeMode();
        leftFollower.setupBrakeMode();
        rightLeader.setupBrakeMode();
        rightFollower.setupBrakeMode();

        leftFollower.setupFollower(Constants.Drivetrain.LEFT_LEADER_PORT, false);
        rightFollower.setupFollower(Constants.Drivetrain.RIGHT_LEADER_PORT, false);

        // all motors should be running in the same direction
        leftLeader.setupInverted(true);
        rightFollower.setupInverted(false);
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

    public double leftLeaderVelocity()
    {
        return leftLeader.getVelocity();
    }

    public double rightLeaderVelocity()
    {
        return rightLeader.getVelocity();
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
        differentialDrive.arcadeDrive(speed.getAsDouble() / divisor, rotation.getAsDouble() / divisor, squared);
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
        differentialDrive.tankDrive(driveSpeed.getAsDouble() / divisor, rotationSpeed.getAsDouble() / divisor, squared);
    }

    /*
     * limits velocity when in high gear to allow shifting to low gear while moving (divisor)
     * temporaily sets motors to coast mode for smoother transition
     */
    public void prepareShiftToLow()
    {
        leftLeader.setupCoastMode();
        rightLeader.setupCoastMode();

        divisor = 1.9;
    }

    /*
     * resets the velocity limitations (divisor)
     * resets motors back to brake mode
     */
    public void postShiftToLow()
    {
        leftLeader.setupBrakeMode();
        rightLeader.setupBrakeMode();

        divisor = 1.0;
    }

    /*
     * temporaily sets motors to coast mode for smoother transition
     */
    public void prepareShiftToHigh()
    {
        leftLeader.setupCoastMode();
        rightLeader.setupCoastMode();
    }

    /*
     * resets motors back to brake mode
     */
    public void postShiftToHigh()
    {
        leftLeader.setupBrakeMode();
        rightLeader.setupBrakeMode();
    }

    /**
     * stops the drivetrain motors
     */
    public void stopDrive()
    {
        differentialDrive.stopMotor();
    }

    public Command leftLeaderVelocityCommand()
    {
        return run( () -> leftLeaderVelocity() ).withName("Get Left Leader Velocity");
    }

    public Command rightLeaderVelocityCommand()
    {
        return run ( () -> rightLeaderVelocity() ).withName("Get Right Leader Velocity");
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
        return run( () -> arcadeDrive(speed, rotation, squared) ).withName("Arcade Drive");
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
        return run( () -> tankDrive(driveSpeed, rotationSpeed, squared) ).withName("Tank Drive");
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
     * the command
     */
    public Command autonomousDriveCommand(double driveSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> 0.0, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand())
            .withName("Autonomous Drive Command");
    }

    /*
     * runs the prepareShiftToLow() method
     */
    public Command prepareShiftToLowCommand()
    {
        return run( () -> prepareShiftToLow()).withName("Prepare Shift to Low");
    }

    /*
     * runs the postShiftToLow() method
     */
    public Command postShiftToLowCommand()
    {
        return run( () -> postShiftToLow()).withName("Shifted to Low");
    }

    /*
     * runs the prepareShiftToHigh() method
     */
    public Command prepareShiftToHighCommmand()
    {
        return run( () -> prepareShiftToHigh()).withName("Prepare Shift to High");
    }

    /*
     * runs the postShiftToHigh() method
     */
    public Command postShiftToHighCommand()
    {
        return run( () -> postShiftToHigh()).withName("Shifted to High");
    }

    /**
     * @param rotationSpeed
     * sets the rotation speed of the motors
     * @param driveTime
     * the amount of time for which the motors will run
     * @return
     * the command
     */
    public Command autonomousTurnCommand(double rotationSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> 0.0, () -> rotationSpeed, false)
            .withTimeout(driveTime)
            .andThen(stopDriveCommand())
            .withName("Autonomous Turn Command");
    }

    /**
     * @param driveSpeed
     * sewts the forward speed of the motors
     * @param rotationSpeed
     * sets the rotation speed of the motors
     * @param driveTime
     * the amount of time for which the motors will run
     * @return
     * the commmand
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
        return "Left Leader Motor Velo = " + leftLeader.getVelocity() + " Left Follower Motor Velo = " + leftFollower.getVelocity() + " Right Leader Motor Velo = " + rightLeader.getVelocity() + " Right Follower Motor Velo = " + rightFollower.getVelocity();
    }
}
