package frc.robot.controls;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.GeneralCommands;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Roller;

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
    private static Roller roller;
    private static Pivot pivot;


    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here

    /**
     * Button bindings for the operator
     * @author Robbie J.
     */

    private OperatorBindings()
    {}

    /**
     * Creates button bindings for the operator
     * @param robotContainer
     * @author Robbie J.
     */

    public static void createBindings(RobotContainer robotContainer)
    {
        System.out.println("Creating Operator Bindings: " + fullClassName);

        controller = robotContainer.getOperatorController();
        roller = robotContainer.getRoller();
        pivot = robotContainer.getPivot();

        if(controller != null)
        {
            configAButton();
            configBButton();
            configXButton();
            configYButton();
            configBackButton();
            configPOVButton();

            configRumble(5);
            configRumble(15);
            if(roller != null)
            {
                // configRumble(() -> DriverStation.isTeleopEnabled() && roller.isDetectedSupplier().getAsBoolean());
            }
        }    
    }

    // private static void configSuppliers()
    // {

    // }

    private static void configAButton()
    {
        Trigger aButtonTrigger = controller.a();
        aButtonTrigger
            .onTrue( GeneralCommands.intakeAlgaeCommand() );
    }
    private static void configBButton()
    {
        Trigger bButtonTrigger = controller.b();
        bButtonTrigger
            .onTrue( GeneralCommands.resetPivotAndRollerCommand() );
    }

    private static void configXButton()
    {
        Trigger xButtonTrigger = controller.x();
        xButtonTrigger
            .onTrue( GeneralCommands.scoreAlgaeCommand() );
    }

    private static void configYButton()
    {
        Trigger yButtonTrigger = controller.y();
        yButtonTrigger
            .onTrue( GeneralCommands.scoreCoralCommand() );
    }

    private static void configBackButton()
    {
        if(pivot != null)
        {
            Trigger backButtonTrigger = controller.back();
            backButtonTrigger
                .onTrue( pivot.resetEncoderCommand() );
        }
        
    }

    private static void configPOVButton()
    {
        if(pivot != null)
        {
            Trigger povUpTrigger = controller.pov(0);
            povUpTrigger
                .onTrue( pivot.moveUpCommand());

            Trigger povDownTrigger = controller.pov(180);
            povDownTrigger
                .onTrue( pivot.moveDownCommand());

            povUpTrigger.negate().and(povDownTrigger.negate())
                .onTrue( pivot.stopCommand());
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
            .onTrue(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0)))
            .onFalse(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
    }

    /**
     * Configures rumble to happen for 0.5 seconds
     * @author Mason B
     */
    // private static void configRumble(BooleanSupplier supplier)
    // {
    //     Trigger rumbleTrigger = new Trigger(supplier);
    //     rumbleTrigger.onTrue(
    //         Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 1.0))
    //         .andThen(Commands.waitSeconds(0.5))
    //         .andThen(Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)))
    //     );
    // }
}

