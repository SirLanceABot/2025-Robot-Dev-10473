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
        // TODO: moveToSetPositionCommand does't ending
        return
        Commands.parallel(
            pivot.moveToSetPositionCommand(TargetPosition.kGrabAlgaePosition),
            roller.intakeCommand(0.5)
        )
        .andThen(pivot.moveToSetPositionCommand(TargetPosition.kStartingPosition));
    }
}
