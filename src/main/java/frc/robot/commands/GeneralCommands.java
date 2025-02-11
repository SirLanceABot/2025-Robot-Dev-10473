package frc.robot.commands;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import frc.robot.RobotContainer;
import frc.robot.sensors.GyroLance;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.TargetPosition;
import frc.robot.subsystems.PoseEstimatorLance;
import frc.robot.subsystems.Roller;


public final class GeneralCommands 
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
    private static Drivetrain drivetrain;
    private static PoseEstimatorLance poseEstimator;
    private static GyroLance gyro;
    private static CommandXboxController driverController;
    private static CommandXboxController operatorController;
    private static LEDs leds;
    private static final InternalButton intakeAlgaeTriggersRumble = new InternalButton();

    private GeneralCommands()
    {}

    public static void createGeneralCommands(RobotContainer robotContainer)
    {
        pivot = robotContainer.getPivot();
        roller = robotContainer.getRoller();
        driverController = robotContainer.getDriverController();
        operatorController = robotContainer.getOperatorController();
        leds = robotContainer.getLEDs();
        intakeAlgaeTriggersRumble.onTrue(operatorRumble());
        
    }

    public static Command resetPivotAndRollerCommand()
    {
        if(pivot != null && roller != null)
        {
            return
                setLEDSolid(Color.kWhite)
                .andThen(pivot.moveToSetPositionCommand(TargetPosition.kStartingPosition)
                    .until( () -> Math.abs(TargetPosition.kStartingPosition.pivot - pivot.getPosition()) < 0.1)
                    .withTimeout(2.0)
                )
                .andThen(
                    Commands.parallel(
                        roller.stopCommand(),
                        operatorRumble(),
                        setLEDOff()
                    )
                );    
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to intake algae from ground
     * @return Command to intake algae
     * @author Mason Bellinger
     * @author Brady Woodard
     */
    public static Command intakeAlgaeCommand()
    {
        if(pivot != null && roller != null)
        {
            return
            Commands.parallel(
                setLEDSolid(Color.kGreen),
                roller.intakeCommand(),
                pivot.moveToSetPositionCommand(TargetPosition.kGrabAlgaePosition)
                    .until( () -> Math.abs(TargetPosition.kGrabAlgaePosition.pivot - pivot.getPosition()) < 0.1)
                    .withTimeout(2.0)
            )
            .andThen(Commands.waitUntil(roller.isDetectedSupplier()))
            .andThen(
                Commands.parallel(
                    roller.stopCommand(),
                    Commands.runOnce( () -> intakeAlgaeTriggersRumble.setPressed(true) ),
                    setLEDSolid(Color.kBlue),
                    pivot.moveToSetPositionCommand(TargetPosition.kStartingPosition)
                        .until( () -> Math.abs(TargetPosition.kStartingPosition.pivot - pivot.getPosition()) < 0.1)
                        .withTimeout(2.0)
                )
            )
            .andThen( () -> intakeAlgaeTriggersRumble.setPressed(false) );
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to score a coral
     * @return Command to score a coral
     * @author Mason Bellinger
     * @author Brady Woodard
     */
    public static Command scoreCoralCommand()
    {
        if(roller != null)
        {
            return
            Commands.parallel(
                setLEDBlink(Color.kRed),
                roller.ejectCoralCommand()
            )
            .withTimeout(2.0)
            .andThen(
                Commands.parallel(
                    operatorRumble(),
                    roller.stopCommand(),
                    setLEDOff()
                )
            );
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to score algae into the processor
     * @return Command to score algae into the processer
     * @author Mason Bellinger
     * @author Brady Woodard
     */
    public static Command scoreAlgaeCommand()
    {
        if(roller != null)
        {
            return
            Commands.parallel(
                setLEDBlink(Color.kBlue),
                roller.ejectAlgaeCommand()
            )
            .withTimeout(2.0)
            .andThen(
                Commands.parallel(
                    operatorRumble(),
                    roller.stopCommand(),
                    setLEDOff()
                )
            );
        }
        else
        {
            return Commands.none();
        }
    }

    // public static Command rotateToNearestScoringLocationCommand()
    // {
    //     if(drivetrain != null && poseEstimator != null && gyro != null)
    //     {
    //         return run()
    //     }
    //     else
    //     {
    //         return Commands.none();
    //     }
    // }

    /**
     * Command to activate rumble on operator controller
     * @return Command to activate rumble on operator controller
     * @author Mason Bellinger
     * @author Brady Woodard
     */
    public static Command operatorRumble()
    {
        if(operatorController != null)
        {
            return
            Commands.runEnd(
                () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1), 
                () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)
            )
            .withTimeout(0.2);
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to activate rumble on driver controller
     * @return Command to activate rumble on driver controller
     * @author Mason Bellinger
     * @author Brady Woodard
     */
    public static Command driverRumble()
    {
        if(driverController != null)
        {
            return
            Commands.runEnd(
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1), 
                () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)
            )
            .withTimeout(0.2);
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to make LEDs a solid color
     * @param color of the LEDs
     * @return command to make LEDs a solid color
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public static Command setLEDSolid(Color color)
    {
        if(leds != null)
        {
            return leds.setColorSolidCommand(color);
            
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to make LEDs blink
     * @param color of the LEDs
     * @return command to make LEDs blink
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public static Command setLEDBlink(Color color)
    {
        if(leds != null)
        {
            return leds.setColorBlinkCommand(color);
            
        }
        else
        {
            return Commands.none();
        }
    }

    /**
     * Command to turn the LEDs off
     * @return command to turn the LEDs off
     * @author Brady Woodard
     * @author Mason Bellinger
     */
    public static Command setLEDOff()
    {
        if(leds != null)
        {
            return leds.offCommand();
            
        }
        else
        {
            return Commands.none();
        }
    }
}
