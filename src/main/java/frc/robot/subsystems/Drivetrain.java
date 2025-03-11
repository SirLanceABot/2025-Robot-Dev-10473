package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.controls.AdaptiveSlewRateLimiter;
import frc.robot.motors.TalonFXLance;
import frc.robot.sensors.GyroLance;

/**
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
    
        private final double FIRST_STAGE_GEAR_RATIO = 12.0 / 60.0;
        private final double SECOND_STAGE_GEAR_RATIO = 24.0 / 32.0;
        private final double LOW_GEAR_RATIO = FIRST_STAGE_GEAR_RATIO * SECOND_STAGE_GEAR_RATIO * (22.0 / 44.0);    // 0.075
        // private final double HIGH_GEAR_RATIO = FIRST_STAGE_GEAR_RATIO * SECOND_STAGE_GEAR_RATIO * (32.0 / 34.0);     // 0.159375
    
        private final double WHEEL_RADIUS_INCHES = 3.0;
        private final double TRACK_WIDTH_INCHES = 21.5; //originally 20.625 // a larger value may be needed to account for wheel slip
        private final double WHEEL_CIRCUMFERENCE_METERS = 2.0 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS_INCHES);
        private final double MOTOR_REVOLUTIONS_TO_WHEEL_METERS = 1.0 / (WHEEL_CIRCUMFERENCE_METERS * LOW_GEAR_RATIO);
        
    
        private final GyroLance gyro;
        // private final PoseEstimatorLance poseEstimator;
    
        private final TalonFXLance leftLeader = new TalonFXLance(Constants.Drivetrain.LEFT_LEADER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Left Leader");
        private final TalonFXLance leftFollower = new TalonFXLance(Constants.Drivetrain.LEFT_FOLLOWER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Left Follower");
        private final TalonFXLance rightLeader = new TalonFXLance(Constants.Drivetrain.RIGHT_LEADER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Right Leader");
        private final TalonFXLance rightFollower = new TalonFXLance(Constants.Drivetrain.RIGHT_FOLLOWER_PORT, Constants.Drivetrain.MOTOR_CAN_BUS, "Right Follower");
    
        private DifferentialDrive differentialDrive = null;
    
        private final AdaptiveSlewRateLimiter xSpeedLimiter = new AdaptiveSlewRateLimiter(Constants.AdaptiveSlewRateLimiter.ACCEL_RATE, Constants.AdaptiveSlewRateLimiter.DECEL_RATE);
        // private final AdaptiveSlewRateLimiter rotationLimiter = new AdaptiveSlewRateLimiter(10.0, 10.0);

        /**
         * PID controller used for PP auto driving
         * First Tune feeds forward kS, then kV, then kP
         */
        private int slotID = 0; // TalonFX internal PID controller
        private double kP = 4.; // TalonFX internal PID controller
        private double kI = 0.;
        private double kD = 0.;
        private double kS = 0.13; // max volts that doesn't move the robot
        private double kV = 3.15; // low gear m/s per volts
    
        private final SimpleMotorFeedforward motorFeedforward = new SimpleMotorFeedforward(kS, kV);
    
        private final DifferentialDriveKinematics kinematics;
    
        private final DifferentialDriveOdometry odometry;
    
        private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        private final StructPublisher<Pose2d> odometryPublisher = networkTable
                .getStructTopic("OdometryPose", Pose2d.struct).publish();
    
        private double divisor = 1.0;
        // divisor is the number to divide the high gear speed outputs by to match the max low gear speed
        // to allow smooth shifting
    
        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        private final MutVoltage m_appliedVoltage = Volts.mutable(0);
        // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
        private final MutDistance m_distance = Meters.mutable(0);
        // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
        private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

        // Create a new SysId routine for characterizing the drive.
        private final SysIdRoutine sysIdRoutine =
          new SysIdRoutine(
              // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
              new SysIdRoutine.Config(),
              new SysIdRoutine.Mechanism(
                  // Tell SysId how to plumb the driving voltage to the motors.
                  voltage -> {
                    leftLeader.setVoltage(voltage.magnitude());
                    leftLeader.feed();
                    rightLeader.setVoltage(voltage.magnitude());
                    rightLeader.feed();
                    differentialDrive.feed();
                  },
                  // Tell SysId how to record a frame of data for each motor on the mechanism being
                  // characterized.
                  log -> {

                    // Record a frame for the left motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-left")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                getLeftLeaderMotorVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(getLeftLeaderDistance(), Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(getLeftLeaderVelocity(), MetersPerSecond));

                    // Record a frame for the right motors.  Since these share an encoder, we consider
                    // the entire group to be one motor.
                    log.motor("drive-right")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                getRightLeaderMotorVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(getRightLeaderDistance(), Meters))
                        .linearVelocity(
                            m_velocity.mut_replace(getRightLeaderVelocity(), MetersPerSecond));

                    // if ((getRightLeaderVelocity() - getRightFollowerVelocity()) / getRightFollowerVelocity() > 0.01)
                    // {
                    //     System.out.println("Right motors: " + getRightLeaderVelocity() + ", " + getRightFollowerVelocity());
                    // }
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));
 
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

        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH_INCHES)); // find track width
        
        odometry = new DifferentialDriveOdometry(
            gyro.getRotation2d(),
            leftLeaderPosition,
            rightLeaderPosition,      // set the gear ratio up
            new Pose2d()
        );

        System.out.println("  Constructor Finished: " + fullClassName);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }
    
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
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

        leftLeader.setupPositionConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);
        rightLeader.setupPositionConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);

        leftLeader.setupVelocityConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);
        leftFollower.setupVelocityConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);
        rightLeader.setupVelocityConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);
        rightFollower.setupVelocityConversionFactor(MOTOR_REVOLUTIONS_TO_WHEEL_METERS);

        setupChassisPIDController(slotID, kP, kI, kD, kS, kV);

        leftLeader.setSafetyEnabled(false);
        rightLeader.setSafetyEnabled(false);

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
     * Setup left and right drive motors with same PID values
     * callable from other classes for tuning purposes
     * 
     * @param slotId
     * @param kP
     * @param kI
     * @param kD
     * @param kS
     * @param kV
     */
    public void setupChassisPIDController(int slotId, double kP, double kI, double kD, double kS, double kV)
    {
        leftLeader.setupPIDController(slotID, kP, kI, kD, kS, kV);
        rightLeader.setupPIDController(slotID, kP, kI, kD, kS, kV);
    }

    /**
     * @return
     * the robot's estimated position of on the field
     */
    public Pose2d getPose()
    {
        // System.out.println("Odometry = " + odometry.getPoseMeters());
        // SmartDashboard.putString("Odometry", odometry.getPoseMeters().toString());
        
        return odometry.getPoseMeters();
    }

    /**
     * resets odometry by reseting the gyro, pose, and the left / right motors
     */
    public void resetOdometry(Pose2d pose)
    {
        odometry.resetPosition(
            gyro.getRotation2d(),
            leftLeaderPosition,
            rightLeaderPosition,
            pose
        );
    }

    /**
     * @return
     * the robot's estimated chassis speed based on the left and right velocities
     */
    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            leftLeaderVelocity,
            rightLeaderVelocity
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

        double leftWheelSpeedInVolts = motorFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightWheelSpeedInVolts = motorFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);

        // System.out.println("-------------------LV = " + leftWheelSpeedInVolts + ", RV = " + rightWheelSpeedInVolts + ", RV = " + rightLeaderVelocity + ", LV = " + leftLeaderVelocity + ", CS = " + chassisSpeeds);
        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
        SmartDashboard.putNumber("Right Volts", rightWheelSpeedInVolts);
        SmartDashboard.putNumber("Left Volts", leftWheelSpeedInVolts);
        SmartDashboard.putNumber("Right velocity", rightLeaderVelocity);
        SmartDashboard.putNumber("Left velocity", leftLeaderVelocity);
        SmartDashboard.putNumber("Left Wheel Speeds", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right Wheel Speeds", wheelSpeeds.rightMetersPerSecond);

        double currentVoltage = RobotController.getBatteryVoltage();
        differentialDrive.tankDrive(leftWheelSpeedInVolts / currentVoltage, rightWheelSpeedInVolts / currentVoltage);

        feedMotors();
    }

    /**
     * Used for PP auto driving
     * @param chassisSpeeds the robot's chassis speed
     * @param feedforwards PP feeds forward - not used
     */
    public void driveRobotRelativeVelocity(ChassisSpeeds chassisSpeeds)
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        // System.out.println("-------------------LV = " + leftWheelSpeedInVolts + ", RV = " + rightWheelSpeedInVolts + ", RV = " + rightLeaderVelocity + ", LV = " + leftLeaderVelocity + ", CS = " + chassisSpeeds);
        SmartDashboard.putString("Chassis Speeds", chassisSpeeds.toString());
        SmartDashboard.putNumber("Left velocity", leftLeaderVelocity);
        SmartDashboard.putNumber("Right velocity", rightLeaderVelocity);
        SmartDashboard.putNumber("Left Wheel Speeds", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right Wheel Speeds", wheelSpeeds.rightMetersPerSecond);

        leftLeader.setControlVelocity(wheelSpeeds.leftMetersPerSecond);
        rightLeader.setControlVelocity(wheelSpeeds.rightMetersPerSecond);

        feedMotors();
    }

    public void feedMotors()
    {
        leftLeader.feed();
        leftFollower.feed();
        rightLeader.feed();
        rightFollower.feed();
        differentialDrive.feed();   
    }
    
    /**
     * @return odometry
     * the robot's estimated position based on the left / right encoders and the gyro
     */
    public DifferentialDriveOdometry getOdometry()
    {
        return odometry;
    }

    public void resetOdometryPose(Pose2d pose)
    {
        odometry.resetPose(pose);
        System.out.println("ResetOdomPose Pose = " + pose);
    }

    /**
     * @return kinematics
     * converts the chassis speed to left and right wheel speeds
     */
    public DifferentialDriveKinematics getKinematics()
    {
        return kinematics;
    }

    /**
     * @return the position of the left wheels [meters]
     */
    public double getLeftLeaderDistance()
    {
        return leftLeaderPosition;
    }

    /**
     * @return the position of the right wheels [meters]
     */
    public double getRightLeaderDistance()
    {
        return rightLeaderPosition;
    }

    /**
     * @return the velocity of the left wheels [meters/second]
     */
    public double getLeftLeaderVelocity()
    {
        return leftLeaderVelocity;
    }

    public double getLeftFollowerVelocity()
    {
        return leftFollowerVelocity;
    }

    /**
     * @return the velocity of the right wheels [meters/second]
     */
    public double getRightLeaderVelocity()
    {
        return rightLeaderVelocity;
    }

    public double getRightFollowerVelocity()
    {
        return rightFollowerVelocity;
    }

    public void resetMotorEncoders()
    {
        rightLeader.setPosition(0.0);
        leftLeader.setPosition(0.0);
        rightFollower.setPosition(0.0);
        leftFollower.setPosition(0.0);
    }

    public double getLeftLeaderDistanceMeters()
    {
        return getRightLeaderDistance() * MOTOR_REVOLUTIONS_TO_WHEEL_METERS;
    }

    public double getRightLeaderDistanceMeters()
    {
        return getLeftLeaderDistance() * MOTOR_REVOLUTIONS_TO_WHEEL_METERS;
    }

    public void setCoastMode()
    {
        leftLeader.setupCoastMode();
        leftFollower.setupCoastMode();
        rightLeader.setupCoastMode();
        rightFollower.setupCoastMode();
    }

    public void setBrakeMode()
    {
        leftLeader.setupBrakeMode();
        leftFollower.setupBrakeMode();
        rightLeader.setupBrakeMode();
        rightFollower.setupBrakeMode();
    }

    /**
     * @return if we should flip out auto paths based on our alliance
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
        differentialDrive.arcadeDrive(speed.getAsDouble() / divisor, rotation.getAsDouble() / divisor / 2.0, squared);
    }   

    public void arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, boolean squared, boolean useSlewRateLimiter)
    {
        double xSpeed;
        // double rotationSpeed;

        if(useSlewRateLimiter)
        {
            xSpeed = xSpeedLimiter.calculate(speed.getAsDouble());
            // rotationSpeed = rotationLimiter.calculate(rotation.getAsDouble());
        }
        else
        {
            xSpeed = speed.getAsDouble();
            // rotationSpeed = rotation.getAsDouble();
        }

        differentialDrive.arcadeDrive(xSpeed, rotation.getAsDouble() / 2.0, squared);

        feedMotors();
        // differentialDrive.arcadeDrive(xSpeed / divisor, rotationSpeed / divisor, squared);
    }  

    // public void fieldRelativeDrive(double xSpeed, double ySpeed)
    // {
    //     double targetRotation, targetSpeed;
    //    
    //     Determines goal angle by finding the arctan of (Y / X)
    //     targetRotation =  Math.atan(ySpeed / xSpeed);
    
    //     Determines goal speed(THis has not shot of working)
    //     targetSpeed = Math.sqrt(Math.pow(ySpeed, 2) + Math.pow(xSpeed, 2));
    // }
    
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
    
        feedMotors();
    }

    /**
     * limits velocity when in high gear to allow shifting to low gear while moving (divisor)
     * temporary sets motors to coast mode for smoother transition
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
     * temporary sets motors to coast mode for smoother transition
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
     * Set both sides of motors to the same speed
     * @param speed %VBus -1 to +1
     */
    public void setDrive(double speed)
    {
        leftLeader.set(speed);
        rightLeader.set(speed);
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

    public Command arcadeDriveCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean squared, boolean useSlewRateLimiter)
    {
        return run( () -> arcadeDrive(speed, rotation, squared, useSlewRateLimiter) ).withName("Arcade Drive");
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

    public Command resetMotorEncodersCommand()
    {
        return runOnce( () -> resetMotorEncoders() ).withName("Reset Motor Encoders");
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

    // /**
    //  * @param rotationSpeed
    //  * sets the rotation speed of the motors
    //  * @param driveTime
    //  * the amount of time for which the motors will run
    //  * @return
    //  * the command
    //  */
    // public Command driveTurnCommand(double rotationSpeed)
    // {
    //     return arcadeDriveCommand(() -> 0.0, () -> rotationSpeed, false).withName("Turn Command");
    // }

    // /**
    //  * @param driveSpeed
    //  * sets speed of arcadeDriveCommand
    //  * @param driveTime
    //  * time for which the Command will run
    //  * @return
    //  * the command
    //  */
    public Command driveStraightCommand(double driveSpeed)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> 0.0, false).withName("Drive Command");
    }

    // /**
    //  * @param driveSpeed
    //  * sets the forward speed of the motors
    //  * @param rotationSpeed
    //  * sets the rotation speed of the motors
    //  * @param driveTime
    //  * the amount of time for which the motors will run
    //  * @return
    //  * the command
    //  */
    public Command turnAndDriveCommand(double driveSpeed, double rotationSpeed)
    {
        return arcadeDriveCommand(() -> driveSpeed, () -> rotationSpeed, false).withName("Turn And Drive Command");
    }

    // public Command snapParallelNearestReefSideCommand(double turnSpeed, double targetRotation)
    // {
    //     // double targetRotation = optimizeRotation(gyro.getYaw(), );
    //     double speed = isRotationSpeedInverted( gyro.getYaw(), targetRotation) ? -turnSpeed : turnSpeed;

    //     return run(() -> driveTurnCommand(speed) )
    //                     .until(isAtRotationSupplier(targetRotation, 2.0)) // tolerance is in degrees
    //                     .withName("Snap To Heading (parallel to reef): " + targetRotation);
    // }

    // public Command snapPerpendicularNearestReefSideCommand(double turnSpeed, double targetRotation)
    // {
    //     // double targetRotation = optimizeRotation(gyro.getYaw(), );
    //     double speed = isRotationSpeedInverted( gyro.getYaw(), targetRotation) ? -turnSpeed : turnSpeed;

    //     return run(() -> driveTurnCommand(speed) )
    //                     .until(isAtRotationSupplier(targetRotation, 2.0)) // tolerance is in degrees
    //                     .withName("Snap To Heading (perpendicular to reef): " + targetRotation);
    // }


    public Command reefBackOutCommand()
    {
        // double starting_pose_meters = getLeftLeaderDistanceMeters();
        System.out.println("Reef Back Out Command");

        return turnAndDriveCommand(-0.2, -1.0)
            .andThen(Commands.print("Reef Back Out Command"))
            .until( () -> (gyro.getRotation2d().getDegrees() > 179.0 && gyro.getRotation2d().getDegrees() < 181.0))
            .withTimeout(10.0);
    }

    public Command driveSpecifiedMetersCommand()
    {
        return driveStraightCommand(1.2);
    }

    public double getLeftLeaderMotorVoltage()
    {
        return leftLeader.getMotorVoltage();
    }

    public double getRightLeaderMotorVoltage()
    {
        return rightLeader.getMotorVoltage();
    }

/********************************************************************************************************************** */

    /**
     * Drivetrain Velocity PID tuning process
     * <p>Control with SmartDashboard interactions
     * <p>Output to SmartDashboard NT
     * <p>Use in DrivetrainTuning
     */

     public class TuneVelocityPID extends Command {

        private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
        static
        {
            System.out.println("Loading: " + fullClassName);
        }
    
        private double velocitySetpoint;
    
        public TuneVelocityPID()
        {
            addRequirements(Drivetrain.this);
        }
    
      public void initialize()
      {
        setName(fullClassName);
        setupChassisPIDController(0, 0.0, 0.0, 0.0, 0.0, 0.0);
        clearGainsSetpoints();
        // set highest possible in case limits are enabled
        // not reset to previous value before PID tuning
        leftLeader.setupCurrentLimit(800., 511., 1.275);
        rightLeader.setupCurrentLimit(800., 511., 1.275);
    }
    
      public void execute()
      {
        if (SmartDashboard.getBoolean("Apply Parameters", false))
        {
            // kF for other PID controllers is essentially this kV and maybe the kS crudely wrapped into it, too.
            // There is also an arbitrary feed-forward that can be used if kS, kV (or others' kF) aren't adequate
            kP = SmartDashboard.getNumber("kP", 0.);
            kI = SmartDashboard.getNumber("kI", 0.);
            kD = SmartDashboard.getNumber("kD", 0.);
            kS = SmartDashboard.getNumber("kS (Rev: 0)", 0.);
            kV = SmartDashboard.getNumber("kV (Rev: kF)", 0.);
            setupChassisPIDController(slotID, kP, kI, kD, kS, kV);
    
            velocitySetpoint = SmartDashboard.getNumber("velocity setpoint [mps]", 0.);
            leftLeader.setControlVelocity(velocitySetpoint);
            rightLeader.setControlVelocity(velocitySetpoint);
    
            SmartDashboard.putBoolean("Apply Parameters", false);
            System.out.println("PID set");
        }
    
        feedMotors();
    
        double tentativeKv = 0.0;
        double avgVelocity = (getLeftLeaderVelocity() + getRightLeaderVelocity()) / 2.0;
        double avgMotorVoltage = (getLeftLeaderMotorVoltage() + getRightLeaderMotorVoltage()) / 2.;
        
        if ( avgVelocity != 0.)
        {
            tentativeKv = (avgMotorVoltage - kS) / avgVelocity;
        }
        SmartDashboard.putNumber("avg velocity [mps] plot", avgVelocity);
        SmartDashboard.putNumber("avg velocity [mps]", avgVelocity);
        SmartDashboard.putNumber("avg motor voltage", avgMotorVoltage);
        SmartDashboard.putNumber("tentative avg Kv", tentativeKv);
        SmartDashboard.putNumber("tentative avg Kv plot", tentativeKv);
      }
    
      public void end(boolean interrupted)
      {
        stopDrive();
      }
    
      /**
       * Whether the command has finished. Once a command finishes, the scheduler will call its end()
       * method and un-schedule it.
       *
       * @return whether the command has finished.
       */
      public boolean isFinished() {
        return false; // always false; let the trigger handle start/stop logic
      }
    
      public boolean runsWhenDisabled() {
        return false;
      }
    
      private void clearGainsSetpoints()
      {
        SmartDashboard.putBoolean("Apply Parameters", false);
        SmartDashboard.putNumber("kP", 0.);
        SmartDashboard.putNumber("kI", 0.);
        SmartDashboard.putNumber("kD", 0.);
        SmartDashboard.putNumber("kS (Rev: 0)", 0.);
        SmartDashboard.putNumber("kV (Rev: kF)", 0.);
        SmartDashboard.putNumber("velocity setpoint [mps]", 0.);
    
        SmartDashboard.putNumber("avg velocity [mps] plot", 0);
        SmartDashboard.putNumber("avg velocity [mps]", 0);
        SmartDashboard.putNumber("avg motor voltage", 0);
        SmartDashboard.putNumber("tentative avg Kv", 0);
        SmartDashboard.putNumber("tentative avg Kv plot", 0);
        SmartDashboard.updateValues();
      }
    }
    /********************************************************************************************************************** */
    
    /********************************************************************************************************************** */

    /**
     * Drivetrain Acceleration Measurement Process
     * <p>Use in DrivetrainTuning
     * <p>Output to SmartDashboard and AdvantageScope NT
     */

     public class MeasureAcceleration extends Command {

        private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
        static
        {
            System.out.println("Loading: " + fullClassName);
        }
            
        private final double stepSize = 1.; // %VBus -1 to 0 to +1
        private final double stopAt = 0.2; // stop motors if below this m/s/s
        private double leftAcceleration;
        private double rightAcceleration;
        private double leftLeaderVelocityPrevious;
        private double rightLeaderVelocityPrevious;
        private double timePrevious;
        private final DoublePublisher voltageStepSizePublisher;
        private final DoublePublisher timePublisher;
        private final DoublePublisher leftLeaderAccelerationPublisher;
        private final DoublePublisher rightLeaderAccelerationPublisher;
        private final DoublePublisher leftLeaderMotorVoltagePublisher;
        private final DoublePublisher rightLeaderMotorVoltagePublisher;
        private final DoublePublisher leftLeaderKaPublisher;
        private final DoublePublisher rightLeaderKaPublisher;
    
        public MeasureAcceleration()
        {
            addRequirements(Drivetrain.this);
            
            var table = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
            voltageStepSizePublisher = table.getDoubleTopic("voltageStepSize").publish();
            timePublisher = table.getDoubleTopic("time").publish();
            leftLeaderAccelerationPublisher = table.getDoubleTopic("leftLeaderAcceleration").publish();
            rightLeaderAccelerationPublisher = table.getDoubleTopic("rightLeaderAcceleration").publish();
            leftLeaderMotorVoltagePublisher = table.getDoubleTopic("leftLeaderMotorVoltage").publish();
            rightLeaderMotorVoltagePublisher = table.getDoubleTopic("rightLeaderMotorVoltage").publish();
            leftLeaderKaPublisher = table.getDoubleTopic("leftLeaderKa").publish();
            rightLeaderKaPublisher = table.getDoubleTopic("rightLeaderKa").publish();
        }
    
        public void initialize()
        {
            setName(fullClassName);

            // set highest possible in case limits are enabled
            // not reset to previous value before PID tuning
            leftLeader.setupCurrentLimit(800., 511., 1.275);
            rightLeader.setupCurrentLimit(800., 511., 1.275);

            // assume starting at rest at time "0"
            leftLeaderVelocityPrevious = 0.;
            rightLeaderVelocityPrevious = 0.;
            timePrevious = Timer.getFPGATimestamp();
            setDrive(stepSize);
            Timer.delay(0.0195); // wait approximately the right time (0.02) to make the first time step otherwise it's immediately hereafter
        }
    
      public void execute()
      {
        var time = Timer.getFPGATimestamp();
        var leftLeaderVelocity = getLeftLeaderVelocity();
        var rightLeaderVelocity = getRightLeaderVelocity();
        var leftLeaderMotorVoltage = getLeftLeaderMotorVoltage();
        var rightLeaderMotorVoltage = getRightLeaderMotorVoltage();

        leftAcceleration =
            (leftLeaderVelocity - leftLeaderVelocityPrevious)
            / (time - timePrevious);
        rightAcceleration =
            (rightLeaderVelocity - rightLeaderVelocityPrevious)
            / (time - timePrevious);

        var leftLeaderKa = leftLeaderMotorVoltage/leftAcceleration;
        var rightLeaderKa = rightLeaderMotorVoltage/rightAcceleration;

        voltageStepSizePublisher.set(stepSize);
        timePublisher.set(time);
        leftLeaderAccelerationPublisher.set(leftAcceleration);
        rightLeaderAccelerationPublisher.set(rightAcceleration);
        leftLeaderMotorVoltagePublisher.set(leftLeaderMotorVoltage);
        rightLeaderMotorVoltagePublisher.set(rightLeaderMotorVoltage);
        leftLeaderKaPublisher.set(leftLeaderKa);
        rightLeaderKaPublisher.set(rightLeaderKa);

        timePrevious = time;
        leftLeaderVelocityPrevious = leftLeaderVelocity;
        rightLeaderVelocityPrevious = rightLeaderVelocity;

        setDrive(stepSize); // first time required, then refresh every time
        feedMotors();
      }
    
      public void end(boolean interrupted)
      {
          stopDrive();
      }
    
      /**
       * Whether the command has finished. Once a command finishes, the scheduler will call its end()
       * method and un-schedule it.
       *
       * @return whether the command has finished.
       */
      public boolean isFinished()
      {
        var finished = false;
        if(leftAcceleration < stopAt || rightAcceleration < stopAt)
        {
            finished = true;
        }
        return finished;
      }
    
      public boolean runsWhenDisabled() {
        return false;
      }
    }
    /********************************************************************************************************************** */

    // *** OVERRIDDEN METHODS ***
    // Put all methods that are Overridden here

    private double leftLeaderPosition;
    private double leftLeaderVelocity;
    private double leftFollowerVelocity;
    private double rightLeaderPosition;
    private double rightLeaderVelocity;
    private double rightFollowerVelocity;

    @Override
    public void periodic()
    {
        leftLeaderVelocity = leftLeader.getVelocity();
        leftFollowerVelocity = leftFollower.getVelocity();
        leftLeaderPosition = leftLeader.getPosition();
        rightLeaderVelocity = rightLeader.getVelocity();
        rightFollowerVelocity = rightFollower.getVelocity();
        rightLeaderPosition = rightLeader.getPosition();
        SmartDashboard.putNumber("LLV", leftLeaderVelocity);
        SmartDashboard.putNumber("LLP", leftLeaderPosition);
        SmartDashboard.putNumber("LFV", leftFollowerVelocity);
        SmartDashboard.putNumber("RLV", rightLeaderVelocity);
        SmartDashboard.putNumber("RLP", rightLeaderPosition);
        SmartDashboard.putNumber("RFV", rightFollowerVelocity);
        Pose2d pose = odometry.update(gyro.getRotation2d(), leftLeaderPosition, rightLeaderPosition);
        odometryPublisher.set(pose);
        
        // System.out.println("Drivetrain periodic pose" + pose);

        // System.out.println("LL = " + leftLeader.getMotorVoltage() + " LF = " + leftFollower.getMotorVoltage() + " RL = " + rightLeader.getMotorVoltage() + " RF = " + rightFollower.getMotorVoltage());
        // System.out.println("LL = " + leftLeader.getMotorSupplyVoltage() + " LF = " + leftFollower.getMotorSupplyVoltage() + " RL = " + rightLeader.getMotorSupplyVoltage() + " RF = " + rightFollower.getMotorSupplyVoltage());
        // System.out.println(this);
    }
        
    @Override
    public String toString()
    {
        return "Left Leader Motor Velo = " + leftLeaderVelocity + " Right Leader Motor Velo = " + rightLeaderVelocity;
        // return "Left = " + leftLeaderPosition + " Right = " + rightLeaderPosition;
    }
}

// var slot0ConfigsNew = slot0Configs.serialize();
// if(!slot0ConfigsNew.equals(slot0ConfigsPrevious))
// {
//     setupChassisPIDController(0, slot0Configs.kP, slot0Configs.kI, slot0Configs.kD, slot0Configs.kS, slot0Configs.kV);
//     slot0ConfigsPrevious = slot0ConfigsNew;
// }

