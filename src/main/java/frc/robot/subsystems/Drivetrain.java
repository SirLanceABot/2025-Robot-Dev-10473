package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.motors.TalonFXLance;
import frc.robot.sensors.GyroLance;

/**
 * @author Robbie Frank 
 * @author Aditya Yadav
 * 
 * makes robot drive
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


    private static final double WHEELRADIUS = 3.0;  // inches
    private static final double TRACKWIDTH = 20.625;  // inches

    private final double FIRSTSTAGEGEARRATIO = 12.0 / 60.0;
    private final double SECONDSTAGEGEARRATIO = 24.0 / 32.0;
    private final double LOWGEARRATIO = FIRSTSTAGEGEARRATIO * SECONDSTAGEGEARRATIO * (22.0 / 44.0);    // 0.075
    private final double HIGHGEARRATIO = FIRSTSTAGEGEARRATIO * SECONDSTAGEGEARRATIO * (32.0 / 34.0);     // 0.159375
    
    private final double MAXLOWGEARSPEED = 0.53125; // where the max high gear speed is 1.0

    // divisor is the number to divide the high gear speed ouputs by to match the max low gear speed
    // to allow smooth shifting
    private double divisor = 1.0;


    private final GyroLance gyro;
    // private final PoseEstimatorLance poseEstimator;

    private final TalonFXLance leftLeader = new TalonFXLance(Constants.Drivetrain.LEFT_LEADER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Left Leader");
    private final TalonFXLance leftFollower = new TalonFXLance(Constants.Drivetrain.LEFT_FOLLOWER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Left Follower");
    private final TalonFXLance rightLeader = new TalonFXLance(Constants.Drivetrain.RIGHT_LEADER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Right Leader");
    private final TalonFXLance rightFollower = new TalonFXLance(Constants.Drivetrain.RIGHT_FOLLOWER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Right Follower");

    private final DifferentialDrive differentialDrive;

    private final DifferentialDriveKinematics kinematics;

    private final DifferentialDriveOdometry odometry;

    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
    private final StructPublisher<Pose2d> odometryPublisher = networkTable
            .getStructTopic("OdometryPose", Pose2d.struct).publish();

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /** 
     * Creates a new Drivetrain. 
     */
    public Drivetrain(GyroLance gyro)
    {
        super("Example Subsystem");
        System.out.println("  Constructor Started:  " + fullClassName);

        this.gyro = gyro;

        configMotors();

        differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TRACKWIDTH)); // find track width
        
        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(),
            leftLeader.getPosition(),
            rightLeader.getPosition(),      // set the gear ratio up
            new Pose2d()
        );

        // try
        // {
        //     RobotConfig config = RobotConfig.fromGUISettings();

        //     AutoBuilder.configure(
        //         this::getPose,
        //         this::resetOdometry,
        //         this::getRobotRelativeSpeeds,
        //         (speeds, feedforwards) -> driveRobotRelative(speeds),
        //         new PPLTVController(0.02),
        //         config,
        //         shouldFlipPath(),
        //         this
        //     );
        // } 
        // catch (Exception e) 
        // {
        //     e.printStackTrace();
        // }

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

        leftLeader.setupPositionConversionFactor(1 / (2.0 * Math.PI * Units.inchesToMeters(WHEELRADIUS) * LOWGEARRATIO));
        rightLeader.setupPositionConversionFactor(1 / (2.0 * Math.PI * Units.inchesToMeters(WHEELRADIUS) * LOWGEARRATIO));

        leftFollower.setSafetyEnabled(false);
        rightFollower.setSafetyEnabled(false);

        leftLeader.setupInverted(false);
        rightLeader.setupInverted(true);

        leftFollower.setupFollower(Constants.Drivetrain.LEFT_LEADER_PORT, false);
        rightFollower.setupFollower(Constants.Drivetrain.RIGHT_LEADER_PORT, false);

        leftLeader.setPosition(0.0);
        rightLeader.setPosition(0.0);

        
        
        // all motors should be running in the same direction
    }

    /**
     * @return
     * the robot's estimated postion of on the field
     */
    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    /**
     * resets odometry by reseting the gryo, pose, and the left / right motors
     */
    public void resetOdometry(Pose2d pose)
    {
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftLeader.getPosition(),
            rightLeader.getPosition(),
            pose
        );
    }

    /**
     * @return
     * the robot's estimated chassis spee based on the left and right velocities
     */
    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            leftLeader.getVelocity(),
            rightLeader.getVelocity()
        );

        return kinematics.toChassisSpeeds(wheelSpeeds);
    }

    /**
     * @param chassisSpeeds
     * the robot's chassis speed
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }
    
    /**
     * @return odometry
     * the robot's esimated position based on the left / right encoders and the gyro
     */
    public DifferentialDriveOdometry getOdometry()
    {
        return odometry;
    }

    public void resetOdometryPose(Pose2d pose)
    {
        odometry.resetPose(pose);
    }

    /**
     * @return kinematics
     * converts the chassi speed to left and right wheel speeds
     */
    public DifferentialDriveKinematics getKinematics()
    {
        return kinematics;
    }

    /**
     * @return the position of the left leader encoder
     */
    public double getLeftLeaderDistance()
    {
        return leftLeader.getPosition();
    }

    /**
     * @return the position of the right reader encoder
     */
    public double getRightLeaderDistance()
    {
        return rightLeader.getPosition();
    }

    /**
     * @return if we should flip out auto pahts based on our alliance
     */
    // public BooleanSupplier shouldFlipPath()
    // {
    //     return 
    //     () -> 
    //     {
    //         Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    //         if(alliance.isPresent()) 
    //         {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //     };
    // }

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

    /**
     * limits velocity when in high gear to allow shifting to low gear while moving (divisor)
     * temporaily sets motors to coast mode for smoother transition
     */
    public void prepareShiftToLow()
    {
        leftLeader.setupCoastMode();
        rightLeader.setupCoastMode();

        divisor = 1.9;  // actual value of high gear to low gear is 1.882352941, rounded to 1.9 for tolerance
    
    }

    /**
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

    /*
     * determines if speed needs to be inverted in order to turn on the most optimal path
     */
    public boolean isRotationSpeedInverted(double currentRotation, double targetRotation)
    {
        double difference = targetRotation - currentRotation;

        return (difference > 0.0 ? true : false);
    }

    public double optimizeRotation(double currentRotation, double tagRotation)
    {
        double positiveRotation = tagRotation + 90.0;
        double negativeRotation = tagRotation - 90.0;
        double difference = Math.abs(positiveRotation - currentRotation);

        return (difference < 90.0 ? positiveRotation : negativeRotation);

        // if(difference < 90.0)
        // {
        //     return positiveRotation;
        // }
        // else
        // {
        //     return negativeRotation;
        // }
    }

    /**
     * stops the drivetrain motors
     */
    public void stopDrive()
    {
        differentialDrive.stopMotor();
    }

    /*
     * returns if the robot's rotation is with the tolerance
     */
    public BooleanSupplier isAtRotationSupplier(double targetRotation, double tolerance)
    {
        return () -> (Math.abs(gyro.getYaw() - targetRotation) < tolerance);
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
     * runs the prepareShiftToLow() method
     */
    public Command prepareShiftToLowCommand()
    {
        return run( () -> prepareShiftToLow()).withName("Prepare Shift to Low");
    }

    /**
     * runs the postShiftToLow() method
     */
    public Command postShiftToLowCommand()
    {
        return run( () -> postShiftToLow()).withName("Shifted to Low");
    }

    /**
     * runs the prepareShiftToHigh() method
     */
    public Command prepareShiftToHighCommand()
    {
        return run( () -> prepareShiftToHigh()).withName("Prepare Shift to High");
    }

    /**
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
    public Command driveTurnCommand(double rotationSpeed)
    {
        return arcadeDriveCommand(() -> 0.0, () -> rotationSpeed, false).withName("Turn Command");
    }

    /**
     * @param driveSpeed
     * sets speed of arcadeDriveCommand
     * @param driveTime
     * time for which the Command will run
     * @return
     * the command
     */
    public Command driveStraightCommand(double driveSpeed)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> 0.0, false).withName("Drive Command");
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
    public Command turnAndDriveCommand(double driveSpeed, double rotationSpeed, double driveTime)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> rotationSpeed, false).withName("Turn And Drive Command");
    }

    public Command snapParallelNearestReefSideCommand(double turnSpeed, double targetRotation)
    {
        // double targetRotation = optimizeRotation(gyro.getYaw(), );
        double speed = isRotationSpeedInverted( gyro.getYaw(), targetRotation) ? -turnSpeed : turnSpeed;

        return run(() -> driveTurnCommand(speed) )
                        .until(isAtRotationSupplier(targetRotation, 2.0)) // tolerance is in degrees
                        .withName("Snap To Heading (parallel to reef): " + targetRotation);
    }

    public Command snapPerpendicularNearestReefSideCommand(double turnSpeed, double targetRotation)
    {
        // double targetRotation = optimizeRotation(gyro.getYaw(), );
        double speed = isRotationSpeedInverted( gyro.getYaw(), targetRotation) ? -turnSpeed : turnSpeed;

        return run(() -> driveTurnCommand(speed) )
                        .until(isAtRotationSupplier(targetRotation, 2.0)) // tolerance is in degrees
                        .withName("Snap To Heading (perpendicular to reef): " + targetRotation);
    }

    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        Pose2d pose = odometry.update(gyro.getRotation2d(), leftLeader.getPosition(), rightLeader.getPosition());
        odometryPublisher.set(pose);
        System.out.println(this);
    }
        
    @Override
    public String toString()
    {
        // return "Left Leader Motor Velo = " + leftLeader.getVelocity() + " Right Leader Motor Velo = " + rightLeader.getVelocity();
        return "Left = " + leftLeader.getPosition() + " Right = " + rightLeader.getPosition();
    }
}
