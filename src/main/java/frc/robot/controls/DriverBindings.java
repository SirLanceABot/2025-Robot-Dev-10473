package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;

public final class DriverBindings
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
    private static CommandXboxController controller;
    private static Drivetrain drivetrain;
    private static Shifter shifter;

    private static DoubleSupplier yAxisSupplier;
    private static DoubleSupplier xAxisSupplier;

    public static final double axisDeadZone = 0.1;

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Button bindings for the driver
     * @author Jackson D.
     */

    private DriverBindings()
    {}

    /**
     * Creates button bindings for the driver
     * @param robotContainer
     * @author Jackson D.
     */
    public static void createBindings(RobotContainer robotContainer)
    {
        System.out.println("Creating Driver Bindings:" + fullClassName);
        
        shifter = robotContainer.getShifter();
        controller = robotContainer.getDriverController();
        drivetrain = robotContainer.getDrivetrain();

        if(controller != null)   
        {  
            configSuppliers();
            configAButton();
            configBButton();
            configXButton();
            // configYButton();
            configLeftBumper();
            // configRightBumper();
            // configBackButton();
            // configStartButton();
            // configLeftStick();
            // configRightStick();
            // configLeftTrigger();
            // configRightTrigger();
            // configDpadUp();
            // configDpadDown();
            configRumble(10);
            configRumble(5);
            configDefaultDrivetrain();
        }
    }

    private static void configSuppliers()
    {
        xAxisSupplier = () -> -controller.getRawAxis(0);
        yAxisSupplier = () -> -controller.getRawAxis(1);
        
        // double xAxis = Math.abs(-controller.getRawAxis(0)) >= axisDeadZone ? -controller.getRawAxis(0) : 0.0;
        // double yAxis = Math.abs(-controller.getRawAxis(4)) >= axisDeadZone ? -controller.getRawAxis(4) : 0.0;

        // xAxisSupplier = () -> xAxis;
        // yAxisSupplier = () -> yAxis;
    }  

    private static void configAButton()
    {
        if(shifter != null)
        {
            Trigger aButtonTrigger = controller.a();
            aButtonTrigger
                .onTrue(shifter.shiftHighCommand());
        }
    }

    private static void configBButton()
    {
        if(shifter != null)
        {
            Trigger bButtonTrigger = controller.b();
            bButtonTrigger
                .onTrue(shifter.shiftLowCommand());
        }    
    }

    private static void configXButton()
    {
        Trigger xButtonTrigger = controller.x();
        xButtonTrigger.onTrue(GeneralCommands.shiftWhileMovingCommand());
    }

    // private static void configYButton()
    // {
    //     Trigger yButtonTrigger = controller.y();
    // }

    private static void configLeftBumper()
    {
        if(shifter != null)
        {
            Trigger leftBumperTrigger = controller.leftBumper();
            leftBumperTrigger
                .onTrue(shifter.shiftToggleCommand());
        }
    }

    // private static void configRightBumper()
    // {
    //     Trigger rightBumperTrigger = controller.rightBumper();
    // }

    // private static void configBackButton()
    // {
    //     Trigger backButtonTrigger = controller.back();
    // }

    // private static void configStartButton()
    // {
    //     Trigger startButtonTrigger = controller.start();
    // }

    // private static void configLeftStick()
    // {
    //     Trigger leftStickTrigger = controller.leftStick();
    // }

    // private static void configRightStick()
    // {
    //     Trigger rightStickTrigger = controller.rightStick();
    // }

    // private static void configLeftTrigger()
    // {}

    // private static void configRightTrigger()
    // {}

    // private static void configDpadUp()
    // {}

    // private static void configDpadDown()
    // {}

    private static void configDefaultDrivetrain()
    {
        if(drivetrain != null)
        {
            drivetrain.setDefaultCommand(drivetrain.arcadeDriveCommand(yAxisSupplier, xAxisSupplier, true) );
        }
    }

    /**
     * Configures rumble to happen at a certain match time on the driver controller
     * @param time The time in the match that the controller rumbles
     * @author Jackson D
     * @author Robbie J
    */ 
    private static void configRumble(int time)
    {
        BooleanSupplier supplier = () -> DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime() - time) < 0.5;
        Trigger rumbleTrigger = new Trigger(supplier);
        rumbleTrigger
            .onTrue(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1)))
            .onFalse(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0)));
    }

}
