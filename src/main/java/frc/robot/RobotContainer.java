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

    private boolean useDrivetrain           = false;

    public final boolean fullRobot;

    private final Drivetrain drivetrain;

    RobotContainer() 
    {
        fullRobot           = (useFullRobot);
        
        drivetrain          = (useFullRobot || useDrivetrain)        ? new Drivetrain()             : null; 

        configureBindings();
    }

    public Drivetrain getDrivetrain()
    {
        return drivetrain;
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
