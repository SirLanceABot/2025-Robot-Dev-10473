// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot 
{
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;
    private TestMode testMode = null;

    public Robot() 
    {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() 
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() 
    {}

    @Override
    public void disabledPeriodic() 
    {}

    @Override
    public void disabledExit() 
    {}

    @Override
    public void autonomousInit() 
    {
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) 
        {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() 
    {}

    @Override
    public void autonomousExit() 
    {}

    @Override
    public void teleopInit() 
    {
        if (m_autonomousCommand != null) 
        {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() 
    {}

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
