package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.TargetPosition;
import frc.robot.subsystems.Roller;


public abstract class GeneralCommands 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private static Pivot pivot;
    private static Roller roller;

    private GeneralCommands()
    {}

    public static void createGeneralCommands(RobotContainer robotContainer)
    {
        pivot = robotContainer.getPivot();
        roller = robotContainer.getRoller();
    }

    public static Command intakeAlgaeCommand()
    {
        if(pivot != null && roller != null)
        {
            return
            Commands.parallel(
                roller.intakeCommand(0.5),
                pivot.moveToSetPositionCommand(TargetPosition.kGrabAlgaePosition)
                    .until( () -> Math.abs(TargetPosition.kGrabAlgaePosition.pivot - pivot.getPosition()) < 0.1)
                    .withTimeout(2.0)
            )
            .andThen(Commands.waitUntil(roller.isDetectedSupplier()))
            .andThen(roller.stopCommand())
            .andThen(pivot.moveToSetPositionCommand(TargetPosition.kStartingPosition)
                .until( () -> Math.abs(TargetPosition.kStartingPosition.pivot - pivot.getPosition()) < 0.1)
                .withTimeout(2.0)            
            );
        }
        else
        {
            return Commands.none();
        }
    }
}
