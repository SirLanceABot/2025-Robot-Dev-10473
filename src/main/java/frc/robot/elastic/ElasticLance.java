package frc.robot.elastic;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Shifter;

public class ElasticLance 
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static Color color = new Color();
    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here

    private ElasticLance()
    {}

    // public static void createWidgets()
    // {
    //     updateAllianceColorBox();
    // }

    public static void sendDataToSmartDashboard()
    {
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.00);
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());

        updateAllianceColorBox();
        updateGearBox();
        // SmartDashboard.putString("Alliance Color", color.toHexString());
        // SmartDashboard.putNumber("Pivot", Pivot.getPosition());

        // SmartDashboard.putNumber(":)", LEDs.getLEDs());
        //SmartDashboard.putBoolean("Alerts Working", Alerts)

        // SmartDashboard.put("Alliance Color", DriverStation.getAlliance());
    }

    public static void updateAllianceColorBox()
    {
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        {
            color = Color.kRed;
        }
        else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        {
            color = Color.kBlue;
        }
        else 
        {
            color = Color.kGray;
        }

        SmartDashboard.putString("Alliance Color", color.toHexString());
    }


    public static void updateGearBox()
    {
        String gear;

        if(Shifter.isHighGear() == true)
        {
            gear = "High";
        }
        else
        {
            gear = "Low";
        }
        // else 
        // {
        //     gear = "None";
        // }

        SmartDashboard.putString("Gear", gear);
    }
}