// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.h

package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.GeneralCommands;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;
import frc.robot.elastic.ElasticLance;
import frc.robot.loggers.DataLogFile;
import frc.robot.motors.MotorControllerLance;
import frc.robot.pathplanner.PathPlannerLance;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pneumatics;

public class Robot extends TimedRobot 
{
    // :)
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final RobotContainer robotContainer;
    private LEDs leds;
    private Drivetrain drivetrain;
    private boolean isPreMatch = true;
    private Command autonomousCommand = null;

    private TestMode testMode = null;
    private Timer timer = new Timer();

    /** 
     * Uses the default access modifier so that the Robot object can only be constructed in this same package.
     */
    Robot() 
    {
        // The order matters here:

        // (1) Configure loggers
        DataLogFile.config();
        // CommandSchedulerLog.config(false, false, false);

        // (2) Create the subsystems, sensors, etc.
        robotContainer = new RobotContainer();
        leds = robotContainer.getLEDs();
        drivetrain = robotContainer.getDrivetrain();

        // (3) Create the commands
        GeneralCommands.createGeneralCommands(robotContainer);

        // (4) Configuring the pathplanner
        PathPlannerLance.configPathPlanner(robotContainer);
        ElasticLance.configElastic(robotContainer);
        // (5) Bind the commands to triggers
        DriverBindings.createBindings(robotContainer);
        OperatorBindings.createBindings(robotContainer);
        // SysIDBindings.createBindings(robotContainer); //FIXME commented out for extra super-duper security against running accidentally

    }

    /**
     * This method runs periodically (20ms) regardless of the mode.
     */
    @Override
    public void robotPeriodic() 
    {
        // Run periodic tasks
        PeriodicTask.runAllPeriodicTasks();

        ElasticLance.sendDataToSmartDashboard();

        // SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization*100.00);
        // SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
        // // SmartDashboard.putNumber("Pivot", Pivot.getPosition());

        // // SmartDashboard.putNumber(":)", LEDs.getLEDs());
        // //SmartDashboard.putBoolean("Alerts Working", Alerts)

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This method runs one time after the driver station connects.
     */
    @Override
    public void driverStationConnected()
    {}

    /**
     * This method runs one time when the robot enters disabled mode.
     */
    @Override
    public void disabledInit() 
    {
        if(leds != null)
        {
           leds.setColorSolid(Color.kRed);
        }
        timer.reset();
        timer.start();
    }

    /**
     * This method runs periodically (20ms) during disabled mode.
     */
    @Override
    public void disabledPeriodic() 
    {
        if(timer.hasElapsed(4.0))
        {
            if(drivetrain != null)
            {
                drivetrain.setCoastMode();
            }
            timer.reset();
            timer.stop();
        }

        if(isPreMatch)
        {
            autonomousCommand = PathPlannerLance.getAutonomousCommand();
            
            if(leds != null)
            {
                if(!robotContainer.useFullRobot())
                {
                    leds.setColorSolid(Color.kRed);
                }
                else if(autonomousCommand.getName().startsWith("1COMP"))
                {
                    leds.setColorSolid(Color.kGreen);
                }
                else
                {
                    leds.setColorSolid(Color.kYellow);
                }
            }    



            // if(currentAutoCommand != previousAutoCommand)
            // {
            //     autonomousCommand = currentAutoCommand;
            // }
            // previousAutoCommand = currentAutoCommand;
        }
    }

    /**
     * This method runs one time when the robot exits disabled mode.
     */
    @Override
    public void disabledExit() 
    {
        if(drivetrain != null)
        {
            drivetrain.setBrakeMode();
        }
    }

    /**
     * This method runs one time when the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() 
    {
        DataLogManager.start();
        // autonomousCommand = PathPlannerLance.getAutonomousCommand();

        if (autonomousCommand != null) 
        {
            autonomousCommand.schedule();
        }

        /**
         * Enables compressor on autonomous init
         */
        Pneumatics pneumatics = robotContainer.getPneumatics();        
        if (pneumatics != null)
        {
            pneumatics.enableCompressor();
        }

        Pivot pivot = robotContainer.getPivot();
        if(pivot != null)
        {
            pivot.resetEncoder();
        }

        if(leds != null)
        {
            leds.off();
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
    {
        if (autonomousCommand != null) 
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }

        isPreMatch = false;
    }

    /**
     * This method runs one time when the robot enters teleop mode.
     */
    @Override
    public void teleopInit() 
    {
        DataLogManager.start();
        
        if (autonomousCommand != null) 
        {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }

        isPreMatch = true;

        if(leds != null)
        {
            leds.off();
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
    {
        MotorControllerLance.logAllStickyFaults();
        DataLogManager.stop();
    }

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
