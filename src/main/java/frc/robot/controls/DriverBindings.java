package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

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

    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /*
     * @author Jackson D.
     */

    private DriverBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        System.out.println("Creating Driver Bindings:" + fullClassName);
        
        controller = robotContainer.getDriverController();

        // configSuppliers();
        // configAButton();
        // configBButton();
        // configXButton();
        // configYButton();
        // configLeftBumper();
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
    }

    private static void configSuppliers()
    {}

    private static void configAButton()
    {
        Trigger aButtonTrigger = controller.a();
    }

    private static void configBButton()
    {
        Trigger bButtonTrigger = controller.b();
    }

    private static void configXButton()
    {
        Trigger xButtonTrigger = controller.x();
    }

    private static void configYButton()
    {
        Trigger yButtonTrigger = controller.y();
    }

    private static void configLeftBumper()
    {
        Trigger leftBumperTrigger = controller.leftBumper();
    }

    private static void configRightBumper()
    {
        Trigger rightBumperTrigger = controller.rightBumper();
    }

    private static void configBackButton()
    {
        Trigger backButtonTrigger = controller.back();
    }

    private static void configStartButton()
    {
        Trigger startButtonTrigger = controller.start();
    }

    private static void configLeftStick()
    {
        Trigger leftStickTrigger = controller.leftStick();
    }

    private static void configRightStick()
    {
        Trigger rightStickTrigger = controller.rightStick();
    }

    private static void configLeftTrigger()
    {}

    private static void configRightTrigger()
    {}

    private static void configDpadUp()
    {}

    private static void configDpadDown()
    {}

    private static void configRumble(int time)
    {
        BooleanSupplier supplier = () -> DriverStation.isTeleopEnabled() && Math.abs(DriverStation.getMatchTime() - time) < 0.5;
        Trigger rumbleTrigger = new Trigger(supplier);
        rumbleTrigger
            .onTrue(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1)))
            .onFalse(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0)));
    }

}
