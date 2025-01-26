// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GeneralCommands;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;

public class Robot extends TimedRobot 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final RobotContainer robotContainer;
    private Command autonomousCommand = null;
    private TestMode testMode = null;

    /** 
     * Uses the default access modifier so that the Robot object can only be constructed in this same package.
     */
    Robot() 
    {
        robotContainer = new RobotContainer();
        GeneralCommands.createGeneralCommands(robotContainer);
        DriverBindings.createBindings(robotContainer);
        OperatorBindings.createBindings(robotContainer);
    }

    /**
     * This method runs periodically (20ms) regardless of the mode.
     */
    @Override
    public void robotPeriodic() 
    {
        // Run periodic tasks
        PeriodicTask.runAllPeriodicTasks();

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This method runs one time when the robot enters disabled mode.
     */
    @Override
    public void disabledInit() 
    {}

    /**
     * This method runs periodically (20ms) during disabled mode.
     */
    @Override
    public void disabledPeriodic() 
    {}

    /**
     * This method runs one time when the robot exits disabled mode.
     */
    @Override
    public void disabledExit() 
    {}

    /**
     * This method runs one time when the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() 
    {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) 
        {
            autonomousCommand.schedule();
        }
    }

    /**
     * This method runs periodically (20ms) during autonomous mode.
     */
    @Override
    public void autonomousPeriodic() 
    {}

    /**
     * This method runs one time when the robot exits autonomous mode.
     */
    @Override
    public void autonomousExit() 
    {}

    /**
     * This method runs one time when the robot enters teleop mode.
     */
    @Override
    public void teleopInit() 
    {
        if (autonomousCommand != null) 
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    /**
     * This method runs periodically (20ms) during teleop mode.
     */
    @Override
    public void teleopPeriodic() 
    {}

    /**
     * This method runs one time when the robot exits teleop mode.
     */
    @Override
    public void teleopExit() 
    {}

    /**
     * This method runs one time when the robot enters test mode.
     */
    @Override
    public void testInit() 
    {
        CommandScheduler.getInstance().cancelAll();

        // Create a TestMode object to test one team members code.
        testMode = new TestMode(robotContainer);

        testMode.init();
    }

    /**
     * This method runs periodically (20ms) during test mode.
     */
    @Override
    public void testPeriodic() 
    {
        testMode.periodic();
    }
    
    /**
     * This method runs one time when the robot exits test mode.
     */
    @Override
    public void testExit() 
    {
        testMode.exit();

        // Set the TestMode object to null so that garbage collection will remove the object.
        testMode = null;
    }
}
