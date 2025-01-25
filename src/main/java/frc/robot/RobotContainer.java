// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shifter;
import frc.robot.commands.GeneralCommands;
import frc.robot.subsystems.Climb;

public class RobotContainer 
{
    // This string gets the full name of the class including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATTIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private boolean useFullRobot            = false;
    private boolean usePivot                = false;
    private boolean useDrivetrain           = false;
    private boolean useRoller               = false;
    private boolean useShifter              = false;
    private boolean useClimb                = false;

    private boolean useDriverController     = false;
    private boolean useOperatorController   = false;

    public final boolean fullRobot;
    private final Pivot pivot;
    private final Drivetrain drivetrain;
    private final Roller roller;
    private final Shifter shifter;
    private final Climb climb;

    private final CommandXboxController operatorController;
    private final CommandXboxController driverController;


    RobotContainer() 
    {
        fullRobot           = (useFullRobot);
        pivot               = (useFullRobot || usePivot)                ? new Pivot()                                                                                                  : null;
        drivetrain          = (useFullRobot || useDrivetrain)           ? new Drivetrain()                                                                                             : null;
        roller              = (useFullRobot || useRoller)               ? new Roller()                                                                                                 : null;
        shifter             = (useFullRobot || useShifter)              ? new Shifter()                                                                                                : null;
        climb               = (useFullRobot || useClimb)                ? new Climb()                                                                                                  : null;

        driverController    = (useFullRobot || useDriverController)     ? new CommandXboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT)                                      : null;
        operatorController  = (useFullRobot || useOperatorController)   ? new CommandXboxController(Constants.Controllers.OPERATOR_CONTROLLER_PORT)                                    : null;
    }

    public Drivetrain getDrivetrain()
    {
        return drivetrain;
    }

    public Pivot getPivot()
    {
        return pivot;
    }

    public Roller getRoller()
    {
        return roller;
    }

    public Shifter getShifter()
    {
        return shifter;
    }

    public Climb getClimb()
    {
        return climb;
    }

    public CommandXboxController getOperatorController()
    {
        return operatorController;
    }

    public CommandXboxController getDriverController()
    {
        return driverController;
    }

    public BooleanSupplier isRedAllianceSupplier()
    {
        return () ->
        {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
            return alliance.get() == DriverStation.Alliance.Red;
            }
            DriverStation.reportError("No alliance is avaliable, assuming Blue", false);
            return false;
        };
    }

    public Command getAutonomousCommand() 
    {
        // return Commands.print("No autonomous command configured");
        return GeneralCommands.intakeAlgaeCommand();
    }
}
