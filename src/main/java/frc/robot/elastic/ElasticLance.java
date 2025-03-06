package frc.robot.elastic;

import java.io.IOException;
import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotContainer;
import frc.robot.pathplanner.PathPlannerLance;
import frc.robot.subsystems.Drivetrain;
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
    private static Field2d autofield = new Field2d();
        
    private static String autoName;
    private static Drivetrain drivetrain;
    private static Shifter shifter;

    private ElasticLance()
    {}

    // public static void createWidgets()
    // {
    //     updateAllianceColorBox();
    // }

    public static void configElastic(RobotContainer robotContainer)
    {
        drivetrain = robotContainer.getDrivetrain();
        shifter = robotContainer.getShifter();
        createAutoField();
    }

    public static void sendDataToSmartDashboard()
    {
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.00);
        SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());

        updateAllianceColorBox();
        updateGearBox();
        //updateAutoField();
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
        String gear = "";

        if(shifter != null)
        {
            if(shifter.isHighGear() == true)
            {
                gear = "High";
            }
            else
            {
                gear = "Low";
            }
        }

        SmartDashboard.putString("Gear", gear);
    }



    private static void createAutoField()
    {
        //Create and push Field2d to SmartDashboard.
        if(drivetrain != null)
        {
            SmartDashboard.putData("AutoField", autofield);
            Pose2d pose = drivetrain.getPose();
            autofield.setRobotPose(pose);
        }
        
    }

    public static void updateAutoField() 
    {
        autoName = PathPlannerLance.getAutonomousCommand().getName();
        List<PathPlannerPath> pathPlannerPaths = null;
        try 
        {
            pathPlannerPaths = getPathPlannerPaths(autoName);
        } catch (IOException | ParseException | org.json.simple.parser.ParseException e) 
        {
            e.printStackTrace();
        }

        if (pathPlannerPaths != null) 
        {
            List<Pose2d> poses = extractPosesFromPaths(pathPlannerPaths);
            autofield.getObject("path").setPoses(poses);
        }    
            
    }                        
                    
                    
    private static List<PathPlannerPath>getPathPlannerPaths(String autoName) throws IOException, ParseException, org.json.simple.parser.ParseException
    {
        return PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    } 



    private static List<Pose2d>extractPosesFromPaths(List<PathPlannerPath> pathPlannerPaths)
    {
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : pathPlannerPaths) 
        {
            poses.addAll(path.getAllPathPoints().stream()
                .map(
                    point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                .collect(Collectors.toList()));
        }
        return poses;
    }



}