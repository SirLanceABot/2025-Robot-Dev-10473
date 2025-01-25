package frc.robot.controls;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;

public final class OperatorBindings 
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


    private OperatorBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        System.out.println("Creating Operator Bindings: " + fullClassName);

        controller = robotContainer.getOperatorController();

        configAButton();
        configBButton();
        configXButton();
        configYButton();
    }

    private static void configSuppliers()
    {

    }

    private static void configAButton()
    {
        Trigger aButtonTrigger = controller.a();
        aButtonTrigger
            .onTrue( GeneralCommands.intakeAlgaeCommand() );
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
}

