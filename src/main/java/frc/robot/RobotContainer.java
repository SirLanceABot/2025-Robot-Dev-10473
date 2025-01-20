// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shifter;
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

    public final boolean fullRobot;
    private final Pivot pivot;
    private final Drivetrain drivetrain;
    private final Roller roller;
    private final Shifter shifter;
    private final Climb climb;

    RobotContainer() 
    {
        fullRobot           = (useFullRobot);
        pivot               = (useFullRobot || usePivot)             ? new Pivot()                  : null;
        drivetrain          = (useFullRobot || useDrivetrain)        ? new Drivetrain()             : null;
        roller              = (useFullRobot || useRoller)            ? new Roller()                 : null;
        shifter             = (useFullRobot || useShifter)           ? new Shifter()                : null;
        climb               = (useFullRobot || useClimb)             ? new Climb()                  : null;

        configureBindings();
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

    private void configureBindings() 
    {}

    public Command getAutonomousCommand() 
    {
        return Commands.print("No autonomous command configured");
    }
}
