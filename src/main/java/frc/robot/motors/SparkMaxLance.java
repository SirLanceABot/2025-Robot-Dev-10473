package frc.robot.motors;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class SparkMaxLance extends MotorControllerLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    @FunctionalInterface
    private interface Function
    {
        public abstract REVLibError doAction();
    }

    // private final CANSparkMax motor;                                         // 2024 version
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkAbsoluteEncoder sparkAbsoluteEncoder = null;
    private SparkLimitSwitch forwardLimitSwitch = null;
    private SparkLimitSwitch reverseLimitSwitch = null;
    // private SparkPIDController sparkPIDController = null;                    // 2024 version
    private SparkClosedLoopController sparkPIDController = null;
    private final String motorControllerName;
    private final ResetMode resetMode = ResetMode.kNoResetSafeParameters;
    private final PersistMode persistMode = PersistMode.kNoPersistParameters;

    private final NetworkTable canSparkMaxTable;
    private StringEntry strEntry;
    // private StringLogEntry motorLogEntry;

    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;

    /**
     * Creates a CANSparkMax on the CANbus with a brushless motor (Neo550 or Neo1650).
     * Defaults to using the built-in encoder sensor (RelativeEncoder).
     * @param deviceId The id number of the device on the CANbus
     * @param canbus The name of the CANbus (ex. "rio" is the default name of the roboRIO CANbus)
     * @param motorControllerName The name describing the purpose of this motor controller
     */
    public SparkMaxLance(int deviceId, String canbus, String motorControllerName)
    {
        super(motorControllerName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + motorControllerName);

        this.motorControllerName = motorControllerName;

        canSparkMaxTable = NetworkTableInstance.getDefault().getTable(Constants.NETWORK_TABLE_NAME);
        strEntry = canSparkMaxTable.getStringTopic("Motors/Setup").getEntry("");
        // strEntry.setDefault("");
        // motorLogEntry = new StringLogEntry(log, "/motors/setup", "Setup");

        motor = new SparkMax(deviceId, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        // sparkPIDController = motor.getPIDController();                       // 2024 version
        sparkPIDController = motor.getClosedLoopController();
        clearStickyFaults();
        setupFactoryDefaults();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Feedback Sensor");
        
        System.out.println("  Constructor Finished: " + fullClassName + " >> " + motorControllerName);
    }

    /** 
     * Check the motor controller for an error and print a message.
     * @param message The message to print
     */
    private void setup(Function func, String message)
    {
        REVLibError errorCode = REVLibError.kOk;
        int attemptCount = 0;
        String logMessage = "";
        
        do
        {
            errorCode = func.doAction();
            logMessage = motorControllerName + " : " + message + " " + errorCode;

            if(errorCode == REVLibError.kOk)
                System.out.println(">> >> " + logMessage);
            else
                DriverStation.reportWarning(logMessage, false);
            
            strEntry.set(logMessage);
            // motorLogEntry.append(logMessage);
            attemptCount++;
        }
        while(errorCode != REVLibError.kOk && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }

    public void burnFlash()
    {
        // motor.burnFlash();
    }

    /**
     * Clear all sticky faults.
     */
    public void clearStickyFaults()
    {
        setup(() -> motor.clearFaults(), "Clear Sticky Faults");
    }

    /**
     * Reset to the factory defaults.
     */
    public void setupFactoryDefaults()
    {
        // setup(() -> motor.restoreFactoryDefaults(false), "Setup Factory Defaults");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        setup(() -> motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters), "Setup Factory Defaults");
    }

    public void setupRemoteCANCoder(int remoteSensorId)
    {
        // sparkAbsoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.fromId(remoteSensorId));
        sparkAbsoluteEncoder = motor.getAbsoluteEncoder();

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Feedback Sensor");
    }

    /**
     * Set the Periodic Frame Period.
     * <p>Defaults: Status0 - 10ms, Status1 - 20ms, Status2 - 20ms, Status3 - 50ms, Status4 - 20ms, Status5 - 200ms, Status6 - 200ms, Status7 - 250ms
     * @param frameNumber The frame number to set
     * @param periodMs The time period in milliseconds
     */
    public void setupPeriodicFramePeriod(int frameNumber, int periodMs)
    {
        // switch(frameNumber)
        // {
        //     case 0:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, periodMs), "Setup Periodic Frame Period 0");
        //         break;
        //     case 1:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, periodMs), "Setup Periodic Frame Period 1");
        //         break;
        //     case 2:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, periodMs), "Setup Periodic Frame Period 2");
        //         break;
        //     case 3:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, periodMs), "Setup Periodic Frame Period 3");
        //         break;
        //     case 4:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, periodMs), "Setup Periodic Frame Period 4");
        //         break;
        //     case 5:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, periodMs), "Setup Periodic Frame Period 5");
        //         break;
        //     case 6:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6, periodMs), "Setup Periodic Frame Period 6");
        //         break;
        //     case 7:
        //         setup(() -> motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus7, periodMs), "Setup Periodic Frame Period 7");
        //         break;
        //     default:
        //         System.out.println("ERROR - Invalid Status Frame Period");
        //         break;
        // }
        motor.setControlFramePeriodMs(periodMs);
    }

    /**
     * Invert the direction of the motor controller.
     * @param isInverted True to invert the motor controller
     */
    public void setupInverted(boolean isInverted)
    {
        // motor.setInverted(isInverted);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.inverted(isInverted);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Inverted");

    }
    
    /**
     * Sets the idle/neutral mode to brake mode.
     */
    public void setupBrakeMode()
    {
        // setup(() -> motor.setIdleMode(IdleMode.kBrake), "Setup Brake Mode");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Brake Mode");
    }
    
    /**
     * Sets the idle/neutral mode to coast mode.
     */
    public void setupCoastMode()
    {
        // setup(() -> motor.setIdleMode(IdleMode.kCoast), "Setup Coast Mode");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kCoast);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Coast Mode");
    }

    /**
     * Set the forward soft limit.
     * @param limit The forward soft limit value
     * @param isEnabled True to enable the forward soft limit
     */
    public void setupForwardSoftLimit(double limit, boolean isEnabled)
    {
        // setup(() -> motor.setSoftLimit(SoftLimitDirection.kForward, (float) limit), "Setup Forward Soft Limit");
        // setup(() -> motor.enableSoftLimit(SoftLimitDirection.kForward, isEnabled), "Enable Forward Soft Limit");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.softLimit.forwardSoftLimit(limit);
        motorConfig.softLimit.forwardSoftLimitEnabled(isEnabled);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Forward Soft Limit");
    }

    /**
     * Set the reverse soft limit.
     * @param limit The reverse soft limit value
     * @param isEnabled True to enable the reverse soft limit
     */
    public void setupReverseSoftLimit(double limit, boolean isEnabled)
    {
        // setup(() -> motor.setSoftLimit(SoftLimitDirection.kReverse, (float) limit), "Setup Reverse Soft Limit");
        // setup(() -> motor.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled), "Enable Reverse Soft Limit");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.softLimit.reverseSoftLimit(limit);
        motorConfig.softLimit.reverseSoftLimitEnabled(isEnabled);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Reverse Soft Limit");
    }

    /**
     * Enable or disable the forward hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     */
    public void setupForwardHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        // if(isNormallyOpen)
        //     forwardLimitSwitch = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // else
        //     forwardLimitSwitch = motor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        // setup(() -> forwardLimitSwitch.enableLimitSwitch(isEnabled), "Setup Forward Hard Limit");

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        if(isNormallyOpen)
            motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
        else
            motorConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Forward Hard Limit");
    }

    /**
     * Enable or disable the reverse hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     */
    public void setupReverseHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        // if(isNormallyOpen)
        //     reverseLimitSwitch = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // else
        //     reverseLimitSwitch = motor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // setup(() -> reverseLimitSwitch.enableLimitSwitch(isEnabled), "Setup Reverse Hard Limit");

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        if(isNormallyOpen)
            motorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        else
            motorConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyClosed);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Reverse Hard Limit");
    }

    /**
     * Set the current limits of the motor.
     * @param currentLimit The current limit in Amps
     * @param currentThreshold The max current limit in Amps
     * @param timeThreshold The time threshold in Seconds
     */
    public void setupCurrentLimit(double currentLimit, double currentThreshold, double timeThreshold)
    {
        // setup(() -> motor.setSmartCurrentLimit((int) currentLimit), "Setup Smart Current Limit");
        // setup(() -> motor.setSecondaryCurrentLimit(currentThreshold, (int) (timeThreshold / 0.00005) ), "Setup Secondary Current Limit");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit((int) currentLimit);
        motorConfig.secondaryCurrentLimit(currentThreshold, (int) (timeThreshold * 20000));
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Current Limit");
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupOpenLoopRampRate(double rampRateSeconds)
    {
        // setup(() -> motor.setOpenLoopRampRate(rampRateSeconds), "Setup Open Loop Ramp Rate");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.openLoopRampRate(rampRateSeconds);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Open Loop Ramp Rate");
    }

    /**
     * Sets the voltage compensation for the motor controller. Use the battery voltage.
     * @param voltageCompensation The nominal voltage to compensate to
     */
    public void setupVoltageCompensation(double voltageCompensation)
    {
        // setup(() -> motor.enableVoltageCompensation(voltageCompensation), "Setup Voltage Compensation");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.voltageCompensation(voltageCompensation);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Voltage Compensation");
    }

    /**
     * Set the conversion factor to convert from sensor position to mechanism position.
     * @param factor The conversion factor to multiply by
     */
    public void setupPositionConversionFactor(double factor)
    {
        // if(sparkAbsoluteEncoder == null)
        //     setup(() -> encoder.setPositionConversionFactor(factor), "Setup Position Conversion Factor");
        // else
        //     setup(() -> sparkAbsoluteEncoder.setPositionConversionFactor(factor), "Setup Position Conversion Factor");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        
        if(sparkAbsoluteEncoder == null)
            motorConfig.encoder.positionConversionFactor(factor);
        else
            motorConfig.absoluteEncoder.positionConversionFactor(factor);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Position Conversion Factor");
    }

    /**
     * Set the conversion factor to convert from sensor velocity to mechanism velocity.
     * @param factor The conversion factor to multiply by
     */
    public void setupVelocityConversionFactor(double factor)
    {
        // if(sparkAbsoluteEncoder == null)
        //     setup(() -> encoder.setVelocityConversionFactor(factor), "Setup Velocity Conversion Factor");
        // else
        //     setup(() -> sparkAbsoluteEncoder.setVelocityConversionFactor(factor), "Setup Velocity Conversion Factor");
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        if(sparkAbsoluteEncoder == null)
            motorConfig.encoder.velocityConversionFactor(factor);
        else
            motorConfig.absoluteEncoder.velocityConversionFactor(factor);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Velocity Conversion Factor");
    }

    /**
     * Set the PID controls for the motor.
     * @param slotID The PID slot (0-3)
     * @param kP The Proportional gain constant
     * @param kI The Integral gain constant
     * @param kD The Derivative gain constant
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD)
    {
        // if(slotId >= 0 && slotId <= 3)
        // {
        //     // set PID coefficients
        //     setup(() -> sparkPIDController.setP(kP, slotId), "Setup PIDController(kP)");
        //     setup(() -> sparkPIDController.setI(kI, slotId), "Setup PIDController(kI)");
        //     setup(() -> sparkPIDController.setD(kD, slotId), "Setup PIDController(kD)");
        // }

        // pidController.setIZone(kIz);
        // pidController.setFF(kFF);
        // pidController.setOutputRange(kMinOutput, kMaxOutput);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

        if(slotId >= 0 && slotId <= 3)
        {
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            motorConfig.closedLoop.pid(kP, kI, kD, closedLoopSlot);
        }
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup PID Controller");
    }

    public double[] getPID(int slotId)
    {
        double[] pid = {0.0, 0.0, 0.0};

        // if(slotId >= 0 && slotId <= 3)
        // {
        //     pid[0] = sparkPIDController.getP(slotId);
        //     pid[1] = sparkPIDController.getI(slotId);
        //     pid[2] = sparkPIDController.getD(slotId);
        // }
        // return pid;

        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
        if(slotId >= 0 & slotId <= 3)
        {
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;
                
            pid[0] = motor.configAccessor.closedLoop.getP(closedLoopSlot);
            pid[1] = motor.configAccessor.closedLoop.getI(closedLoopSlot);
            pid[2] = motor.configAccessor.closedLoop.getD(closedLoopSlot);
        }

        return pid;
    }
    
    /**
     * Sets a motor to be a follower of another motor.
     * Setting the power of the leader, also sets the power of the follower.
     * @param leaderId The id of the leader motor on the can bus
     * @param isInverted True to invert the motor so it runs opposite of the leader
     */
    public void setupFollower(int leaderId, boolean isInverted)
    {
        // setup(() -> motor.follow(CANSparkBase.ExternalFollower.kFollowerSpark, leaderId, isInverted), "Setup Follower");
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.follow(leaderId, isInverted);
        setup(() -> motor.configure(motorConfig, resetMode, persistMode), "Setup Follower");
    }

    /**
     * @return true if limit switch is pressed
     */
    public boolean isForwardLimitSwitchPressed()
    {
        return forwardLimitSwitch.isPressed();
    }

    /**
     * @return true if limit switch is pressed
     */
    public boolean isReverseLimitSwitchPressed()
    {
        return reverseLimitSwitch.isPressed();
    }

    /**
     * Logs the sticky faults
     */
    public void logStickyFaults()
    {
        // int faults = motor.getStickyFaults();
        // strEntry = canSparkMaxTable.getStringTopic("Motors/Faults").getEntry("");
        // // motorLogEntry = new StringLogEntry(log, "/motors/faults", "Faults");

        // if(faults > 0)
        // {
        //     for(int i = 0; i < 16; i++)
        //     {
        //         if((faults & (1 << i)) > 0)
        //             strEntry.set(motorControllerName + " : " + CANSparkBase.FaultID.fromId(i));
        //             // motorLogEntry.append(motorControllerName + " : " + CANSparkBase.FaultID.fromId(i));
        //     }
        // }
        // else
        //     strEntry.set(motorControllerName + " : No Sticky Faults");
        //     // motorLogEntry.append(motorControllerName + " : No Sticky Faults");

        // clearStickyFaults();

        int faultsCount = 0;
        Faults faults = motor.getStickyFaults();
        Warnings warnings = motor.getStickyWarnings();
        strEntry = canSparkMaxTable.getStringTopic("Motors/Faults").getEntry("");

        if(faults.can)
        {
            strEntry.set(motorControllerName + " : Fault - CAN");
            // motorLogEntry.append(motorControllerName + " : Fault - CAN");
            faultsCount++;
        }
        if(faults.sensor)
        {
            strEntry.set(motorControllerName + " : Fault - Sensor");
            // motorLogEntry.append(motorControllerName + " : Fault - Sensor");
            faultsCount++;
        }
        if(faults.temperature)
        {
            strEntry.set(motorControllerName + " : Fault - Temperature");
            // motorLogEntry.append(motorControllerName + " : Fault - Temperature");
            faultsCount++;
        }

        if(warnings.brownout)
        {
            strEntry.set(motorControllerName + " : Warning - Brownout");
            // motorLogEntry.append(motorControllerName + " : Warning - Brownout");
            faultsCount++;
        }
        if(warnings.hasReset)
        {
            strEntry.set(motorControllerName + " : Warning - Has Reset");
            // motorLogEntry.append(motorControllerName + " : Warning - Has Reset");
            faultsCount++;
        }
        if(warnings.overcurrent)
        {
            strEntry.set(motorControllerName + " : Warning - Overcurrent");
            // motorLogEntry.append(motorControllerName + " : Warning - Overcurrent");
            faultsCount++;
        }
        if(warnings.stall)
        {
            strEntry.set(motorControllerName + " : Warning - Stall");
            // motorLogEntry.append(motorControllerName + " : Warning - Stall");
            faultsCount++;
        }

        if(faultsCount == 0)
        {
            strEntry.set(motorControllerName + " : No Sticky Faults");
            // motorLogEntry.append(motorControllerName + " : No Sticky Faults");
        }

        clearStickyFaults();
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * Uses PID slot 0 by default.
     * @param position The position to move the motor to
     */
    public void setControlPosition(double position)
    {
        // sparkPIDController.setReference(position, CANSparkBase.ControlType.kPosition);
        setControlPosition(position, 0);
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position to move the motor to
     * @param slotID The PID slot (0-3)
     */
    public void setControlPosition(double position, int slotId)
    {
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

        if(slotId >= 0 && slotId <= 3)
        {
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            sparkPIDController.setReference(position, ControlType.kPosition, closedLoopSlot);
        }
    }

    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * Uses PID slot 0 by default.
     * @param velocity The velocity to spin the motor at
     */
    public void setControlVelocity(double velocity)
    {
        // sparkPIDController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
        setControlVelocity(velocity, 0);
    }

    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param velocity The velocity to spin the motor at
     * @param slotID The PID slot (0-3)
     */
    public void setControlVelocity(double velocity, int slotId)
    {
        ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

        if(slotId >= 0 && slotId <= 3)
        {
            if(slotId == 0)
                closedLoopSlot = ClosedLoopSlot.kSlot0;
            else if(slotId == 1)
                closedLoopSlot = ClosedLoopSlot.kSlot1;
            else if(slotId == 2)
                closedLoopSlot = ClosedLoopSlot.kSlot2;
            else if(slotId == 3)
                closedLoopSlot = ClosedLoopSlot.kSlot3;

            sparkPIDController.setReference(velocity, ControlType.kVelocity, closedLoopSlot);
        }
    }

    /**
     * Set the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position of the encoder
     */
    public void setPosition(double position)
    {
        encoder.setPosition(position);
    }

    /**
     * Get the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @return The position of the encoder
     */
    public double getPosition()
    {
        if(sparkAbsoluteEncoder == null)
            return encoder.getPosition();
        else
            return sparkAbsoluteEncoder.getPosition();
    }

    /**
     * Get the velocity of the encoder.
     * Units are RPMs by default, but can be changed using the conversion factor.
     * @return The velocity of the encoder
     */    
    public double getVelocity()
    {
        if(sparkAbsoluteEncoder == null)
            return encoder.getVelocity();
        else
            return sparkAbsoluteEncoder.getVelocity();
    }

    /**
     * Get the applied motor voltage (in volts).
     * @return The voltage
     */    
    public double getMotorVoltage()
    {
        return motor.getBusVoltage();
    }

    @Override
    public void stopMotor()
    {
        set(0.0);
    }

    @Override
    public String getDescription()
    {
        return motorControllerName;
    }

    @Override
    public void set(double speed)
    {
        motor.set(speed);
        feed();
    }

    @Override
    public void setVoltage(double outputVolts) 
    {
        motor.setVoltage(outputVolts);
        feed();
    }

    @Override
    public double get()
    {
        return motor.get();
    }

    /**
     * @deprecated Use <b>setupInverted()</b> instead
     */
    @Override
    public void setInverted(boolean isInverted)
    {
        setupInverted(isInverted);
    }

    @Override
    public boolean getInverted()
    {
        // return motor.getInverted();
        return motor.configAccessor.getInverted();
    }

    @Override
    public void disable()
    {
        motor.disable();
    }
}
