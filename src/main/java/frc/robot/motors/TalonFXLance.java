package frc.robot.motors;

import java.lang.invoke.MethodHandles;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.spns.SpnValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DriverStation;

public class TalonFXLance extends MotorControllerLance
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
        public abstract StatusCode doAction();
    }

    private final TalonFX motor;
    private final PositionVoltage positionVoltage;
    private final VelocityVoltage velocityVoltage;
    private final String motorControllerName;

    private final int SETUP_ATTEMPT_LIMIT = 5;
    private int setupErrorCount = 0;

    /**
     * Creates a TalonFX on the CANbus with a brushless motor (Falcon500).
     * Defaults to using the built-in encoder sensor (RotorSensor).
     * @param deviceId The id number of the device on the CANbus
     * @param canbus The name of the CANbus (ex. "rio" is the default name of the roboRIO CANbus)
     * @param motorControllerName The name describing the purpose of this motor controller
     */
    public TalonFXLance(int deviceId, String canbus, String motorControllerName)
    {
        super(motorControllerName);

        System.out.println("  Constructor Started:  " + fullClassName + " >> " + motorControllerName);

        this.motorControllerName = motorControllerName;

        motor = new TalonFX(deviceId, canbus);
        positionVoltage = new PositionVoltage(0.0);
        velocityVoltage = new VelocityVoltage(0);

        clearStickyFaults();
        setupFactoryDefaults();
        setupFeedbackSensor();

        System.out.println("  Constructor Finished: " + fullClassName + " >> " + motorControllerName);
    }

    /** 
     * Check the motor controller for an error and print a message.
     * @param message The message to print
     */
    private void setup(Function func, String message)
    {
        StatusCode errorCode = StatusCode.OK;
        int attemptCount = 0;
        String logMessage = "";
        
        do
        {
            errorCode = func.doAction();
            logMessage = motorControllerName + " : " + message + " " + errorCode;

            if(errorCode == StatusCode.OK)
                System.out.println(">> >> " + logMessage);
            else
                DriverStation.reportWarning(logMessage, true);

            motorSetupPublisher.set(logMessage);

            attemptCount++;
        }
        while(errorCode != StatusCode.OK && attemptCount < SETUP_ATTEMPT_LIMIT);

        setupErrorCount += (attemptCount - 1);
    }

    /**
     * Setup feedback sensor to built-in encoder
     */
    private void setupFeedbackSensor()
    {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        setup(() -> motor.getConfigurator().refresh(feedbackConfigs), "Refresh Feedback Sensor");
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        setup(() -> motor.getConfigurator().apply(feedbackConfigs), "Setup Feedback Sensor");
    }

    /**
     * Clear all sticky faults.
     */
    public void clearStickyFaults()
    {
        setup(() -> motor.clearStickyFaults(), "Clear Sticky Faults");
    }

    /**
     * Reset to the factory defaults.
     */
    public void setupFactoryDefaults()
    {
        setup(() -> motor.getConfigurator().apply(new TalonFXConfiguration()), "Setup Factory Defaults");
    }

    public void setupRemoteCANCoder(int remoteSensorId)
    {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        setup(() -> motor.getConfigurator().refresh(feedbackConfigs), "Refresh Remote CANCoder");
    
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = remoteSensorId;
        setup(() -> motor.getConfigurator().apply(feedbackConfigs), "Setup Remote CANCoder");
    }

    /**
     * Set the Periodic Frame Period.
     * @param frameNumber The frame number to set
     * @param periodMs The time period in milliseconds
     */
    public void setupPeriodicFramePeriod(int frameNumber, int periodMs)
    {
        // FIXME
    }

    /**
     * Invert the direction of the motor controller.
     * @param isInverted True to invert the motor controller
     */
    public void setupInverted(boolean isInverted)
    {
        // motor.setInverted(isInverted);
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        setup(() -> motor.getConfigurator().refresh(motorOutputConfigs), "Refresh Inverted");

        if(isInverted)
            motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        else
            motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        setup(() -> motor.getConfigurator().apply(motorOutputConfigs), "Setup Inverted");
    }

    /**
     * Sets the idle/neutral mode to brake mode.
     */
    public void setupBrakeMode()
    {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        setup(() -> motor.getConfigurator().refresh(motorOutputConfigs), "Refresh Brake Mode");
        
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        setup(() -> motor.getConfigurator().apply(motorOutputConfigs), "Setup Brake Mode");
    }

    /**
     * Sets the idle/neutral mode to coast mode.
     */
    public void setupCoastMode()
    {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        setup(() -> motor.getConfigurator().refresh(motorOutputConfigs), "Refresh Coast Mode");

        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        setup(() -> motor.getConfigurator().apply(motorOutputConfigs), "Setup Coast Mode");
    }

    /**
     * Set the forward soft limit.
     * @param limit The forward soft limit value
     * @param isEnabled True to enable the forward soft limit
     */
    public void setupForwardSoftLimit(double limit, boolean isEnabled)
    {
        SoftwareLimitSwitchConfigs softLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        setup(() -> motor.getConfigurator().refresh(softLimitSwitchConfigs), "Refresh Forward Soft Limit");

        softLimitSwitchConfigs.ForwardSoftLimitThreshold = limit;
        softLimitSwitchConfigs.ForwardSoftLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(softLimitSwitchConfigs), "Setup Forward Soft Limit");
    }

    /**
     * Set the reverse soft limit.
     * @param limit The reverse soft limit value
     * @param isEnabled True to enable the reverse soft limit
     */
    public void setupReverseSoftLimit(double limit, boolean isEnabled)
    {
        SoftwareLimitSwitchConfigs softLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        setup(() -> motor.getConfigurator().refresh(softLimitSwitchConfigs), "Refresh Reverse Soft Limit");

        softLimitSwitchConfigs.ReverseSoftLimitThreshold = limit;
        softLimitSwitchConfigs.ReverseSoftLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(softLimitSwitchConfigs), "Setup Reverse Soft Limit");
    }

    /**
     * Enable or disable the forward hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     */
    public void setupForwardHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        setup(() -> motor.getConfigurator().refresh(hardwareLimitSwitchConfigs), "Refresh Forward Hard Limit");

        hardwareLimitSwitchConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
        if(isNormallyOpen)
            hardwareLimitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
        else
            hardwareLimitSwitchConfigs.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
        hardwareLimitSwitchConfigs.ForwardLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(hardwareLimitSwitchConfigs), "Setup Forward Hard Limit");
    }

    /**
     * Enable or disable the reverse hard limit switch.
     * @param isEnabled True to enable the hard limit switch
     */
    public void setupReverseHardLimitSwitch(boolean isEnabled, boolean isNormallyOpen)
    {
        HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs = new HardwareLimitSwitchConfigs();
        setup(() -> motor.getConfigurator().refresh(hardwareLimitSwitchConfigs), "Refresh Reverse Hard Limit");

        hardwareLimitSwitchConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
        if(isNormallyOpen)
            hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        else
            hardwareLimitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
        hardwareLimitSwitchConfigs.ReverseLimitEnable = isEnabled;
        setup(() -> motor.getConfigurator().apply(hardwareLimitSwitchConfigs), "Setup Reverse Hard Limit");
    }

    /**
     * Set the current limits of the motor.
     * @param currentLimit The current limit in Amps
     * @param currentThreshold The max current limit in Amps
     * @param timeThreshold The time threshold in Seconds
     */
    public void setupCurrentLimit(double currentLimit, double currentThreshold, double timeThreshold)
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        setup(() -> motor.getConfigurator().refresh(currentLimitsConfigs), "Refresh Current Limit");

        currentLimitsConfigs.SupplyCurrentLimit = currentLimit;
        // currentLimitsConfigs.SupplyCurrentThreshold = currentThreshold;          // 2024 version
        currentLimitsConfigs.SupplyCurrentLowerLimit = currentThreshold;
        // currentLimitsConfigs.SupplyTimeThreshold = timeThreshold;                // 2024 version
        currentLimitsConfigs.SupplyCurrentLowerTime = timeThreshold;
        setup(() -> motor.getConfigurator().apply(currentLimitsConfigs), "Setup Current Limit");
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupOpenLoopRampRate(double rampRateSeconds)
    {
        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        setup(() -> motor.getConfigurator().refresh(openLoopRampsConfigs), "Refresh Open Loop Ramp Rate");

        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = rampRateSeconds;
        setup(() -> motor.getConfigurator().apply(openLoopRampsConfigs), "Setup Open Loop Ramp Rate");
    }

    /**
     * Set the maximum rate at which the motor output can change.
     * @param rampRateSeconds Time in seconds to go from 0 to full throttle
     */
    public void setupClosedLoopRampRate(double rampRateSeconds)
    {
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        setup(() -> motor.getConfigurator().refresh(closedLoopRampsConfigs), "Refresh Closed Loop Ramp Rate");

        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = rampRateSeconds;
        setup(() -> motor.getConfigurator().apply(closedLoopRampsConfigs), "Setup Closed Loop Ramp Rate");
    }

    /**
     * Sets the voltage compensation for the motor controller. Use the battery voltage.
     * @param voltageCompensation The nominal voltage to compensate to
     */
    public void setupVoltageCompensation(double voltageCompensation)
    {
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        setup(() -> motor.getConfigurator().refresh(voltageConfigs), "Refresh Voltage Compensation");

        voltageConfigs.PeakForwardVoltage = voltageCompensation;
        voltageConfigs.PeakReverseVoltage = -voltageCompensation;
        setup(() -> motor.getConfigurator().apply(voltageConfigs), "Setup Voltage Compensation");
    }

    /**
     * Set the conversion factor to convert from sensor rotations to mechanism output.
     * @param factor The conversion factor to multiply by
     */
    public void setupPositionConversionFactor(double factor)
    {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        setup(() -> motor.getConfigurator().refresh(feedbackConfigs), "Refresh Position Conversion Factor");

        feedbackConfigs.SensorToMechanismRatio = factor;
        setup(() -> motor.getConfigurator().apply(feedbackConfigs), "Setup Position Conversion Factor");
    }

    /**
     * Set the conversion factor to convert from sensor velocity to mechanism velocity.
     * @param factor The conversion factor to multiply by
     */
    public void setupVelocityConversionFactor(double factor)
    {
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        setup(() -> motor.getConfigurator().refresh(feedbackConfigs), "Refresh Velocity Conversion Factor");

        feedbackConfigs.SensorToMechanismRatio = factor;
        setup(() -> motor.getConfigurator().apply(feedbackConfigs), "Setup Velocity Conversion Factor");
    }

    /**
     * Set the PID controls for the motor.
     * @param kP The Proportional constant
     * @param kI The Integral constant
     * @param kD The Derivative constant
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD)
    {
        if(slotId >= 0 && slotId <= 2)
        {
            SlotConfigs slotConfigs = new SlotConfigs();
            setup(() -> motor.getConfigurator().refresh(slotConfigs), "Refresh PID Controller");

            slotConfigs.SlotNumber = slotId;
            slotConfigs.kP = kP;
            slotConfigs.kI = kI;
            slotConfigs.kD = kD;
            setup(() -> motor.getConfigurator().apply(slotConfigs), "Setup PID Controller"); 
        }
    }

    /**
     * Set the PID controls for the motor.
     * @param slotID The PID slot (0-2)
     * @param kP The Proportional constant
     * @param kI The Integral constant
     * @param kD The Derivative constant
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD, double kS, double kV)
    {
        if(slotId >= 0 && slotId <= 2)
        {
            SlotConfigs slotConfigs = new SlotConfigs();
            setup(() -> motor.getConfigurator().refresh(slotConfigs), "Refresh PID Controller");

            slotConfigs.SlotNumber = slotId;
            slotConfigs.kP = kP;
            slotConfigs.kI = kI;
            slotConfigs.kD = kD;
            slotConfigs.kS = kS;
            slotConfigs.kV = kV;
            setup(() -> motor.getConfigurator().apply(slotConfigs), "Setup PID Controller"); 
        }
    }

    /**
     * Set the PID controls for the motor.
     * @param slotID The PID slot (0-2)
     * @param kP The Proportional constant
     * @param kI The Integral constant
     * @param kD The Derivative constant
     * @param kS Velocity feedforward gain
     * @param kV Acceleration feedforward gain
     * @param kA Gravity feedforward/feedback gain
     * @param kG Gravity Feedforward/Feedback Type.
     * @param gravType Elevator_Static for constant equal gravity, Arm_Cosine for when gravity applies varying force
     */
    public void setupPIDController(int slotId, double kP, double kI, double kD, double kS, double kV, double kA, double kG, GravityTypeValue gravType)
    {
        SlotConfigs slotConfigs = new SlotConfigs();
        setup(() -> motor.getConfigurator().refresh(slotConfigs), "Refresh PID Controller");

        slotConfigs.SlotNumber = slotId;
        slotConfigs.kP = kP;
        slotConfigs.kI = kI;
        slotConfigs.kD = kD;
        slotConfigs.kS = kS;
        slotConfigs.kV = kV;
        slotConfigs.kA = kA;
        slotConfigs.kG = kG;
        slotConfigs.GravityType = gravType;
        setup(() -> motor.getConfigurator().apply(slotConfigs), "Setup PID Controller");
    }

    public double[] getPID(int slotId)
    {
        double[] pid = {0.0, 0.0, 0.0};
        if(slotId >= 0 && slotId <= 2)
        {
            SlotConfigs slotConfigs = new SlotConfigs();
            switch(slotId)
            {
                case 0:
                    setup(() -> motor.getConfigurator().refresh(Slot0Configs.from(slotConfigs)), "Refresh PID Slot0 Configs");
                    break;
                case 1:
                    setup(() -> motor.getConfigurator().refresh(Slot1Configs.from(slotConfigs)), "Refresh PID Slot1 Configs");
                    break;
                case 2:
                    setup(() -> motor.getConfigurator().refresh(Slot2Configs.from(slotConfigs)), "Refresh PID Slot2 Configs");
                    break;
            }     
                
            pid[0] = slotConfigs.kP;
            pid[1] = slotConfigs.kI;
            pid[2] = slotConfigs.kD;
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
        setup(() -> motor.setControl(new Follower(leaderId, isInverted)), "Setup Follower");
    }

    /**
     * Logs the sticky faults
     */
    public void logStickyFaults()
    {
        // int faults = motor.getStickyFaultField().getValue();
        int faultsCount = 0;

        if(setupErrorCount > 0)
        {
            motorSetupPublisher.set(motorControllerName + " : " + setupErrorCount + " setup errors");
        }

        if(motor.getStickyFault_BootDuringEnable().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_BootDuringEnable);
            faultsCount++;
        }
        if(motor.getStickyFault_BridgeBrownout().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_BridgeBrownout);
            faultsCount++;
        }
        if(motor.getStickyFault_DeviceTemp().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_DeviceTemp);
            faultsCount++;
        }
        if(motor.getStickyFault_ForwardHardLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_ForwardHardLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_ForwardSoftLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_ForwardSoftLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_FusedSensorOutOfSync().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_FusedSensorOutOfSync);
            faultsCount++;
        }
        if(motor.getStickyFault_Hardware().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_Hardware);
            faultsCount++;
        }
        if(motor.getStickyFault_MissingDifferentialFX().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_MissingDifferentialFX);
            faultsCount++;
        }
        if(motor.getStickyFault_OverSupplyV().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_OverSupplyV);
            faultsCount++;
        }
        if(motor.getStickyFault_RemoteSensorDataInvalid().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_RemoteSensorPosOverflow);
            faultsCount++;
        }
        if(motor.getStickyFault_RemoteSensorPosOverflow().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_RemoteSensorPosOverflow);
            faultsCount++;
        }
        if(motor.getStickyFault_RemoteSensorReset().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_RemoteSensorReset);
            faultsCount++;
        }
        if(motor.getStickyFault_ReverseHardLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_ReverseHardLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_ReverseSoftLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_ReverseSoftLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_StatorCurrLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_StatorCurrLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_SupplyCurrLimit().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_SupplyCurrLimit);
            faultsCount++;
        }
        if(motor.getStickyFault_Undervoltage().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_Undervoltage);
            faultsCount++;
        }
        if(motor.getStickyFault_UnlicensedFeatureInUse().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_UnlicensedFeatureInUse);
            faultsCount++;
        }
        if(motor.getStickyFault_UnstableSupplyV().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_UnstableSupplyV);
            faultsCount++;
        }
        if(motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue())
        {
            motorFaultsPublisher.set(motorControllerName + " : " + SpnValue.StickyFault_TALONFX_UsingFusedCCWhileUnlicensed);
            faultsCount++;
        }

        if(faultsCount == 0)
        {
            motorFaultsPublisher.set(motorControllerName + " : No Sticky Faults");
        }

        clearStickyFaults();
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position to move the motor to
     */
    public void setControlPosition(double position)
    {
        motor.setControl(positionVoltage.withPosition(position));
    }

    /**
     * Move the motor to a position using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position to move the motor to
     * @param slotID The PID slot (0-2)
     */
    public void setControlPosition(double position, int slotId)
    {
        if(slotId >= 0 && slotId <= 2)
        {
            motor.setControl( positionVoltage
                .withPosition(position)
                .withSlot(slotId)
            );
        }
    }


    /**
     * Spin the motor to a velocity using PID control.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param velocity The velocity to spin the motor at
     */
    public void setControlVelocity(double velocity)
    {
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }

    /**
     * Set the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @param position The position of the encoder
     */
    public void setPosition(double position)
    {
        motor.setPosition(position);
    }

    /**
     * Get the position of the encoder.
     * Units are rotations by default, but can be changed using the conversion factor.
     * @return The position of the encoder
     */
    public double getPosition()
    {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Get the velocity of the encoder.
     * Units are RPS by default, but can be changed using the conversion factor.
     * @return The velocity of the encoder
     */    
    public double getVelocity()
    {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the applied motor voltage (in volts).
     * @return The voltage
     */    
    public double getMotorVoltage()
    {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public double getMotorSupplyVoltage()
    {
        return motor.getSupplyVoltage().getValueAsDouble();
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

    @Override
    public boolean getInverted()
    {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        return motorOutputConfigs.Inverted == InvertedValue.Clockwise_Positive;
    }

    @Override
    public void disable()
    {
        motor.disable();
    }
}
