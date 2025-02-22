// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.sensors.CameraLL;
import frc.robot.sensors.GyroLance;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.PoseEstimatorLance;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shifter;

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
    private boolean usePneumatics           = false;
    private boolean useClimb                = false;
    private boolean useLEDs                 = false;
    private boolean usePoseEstimator        = false;
    private boolean useCamera               = false;

    private boolean useDriverController     = false;
    private boolean useOperatorController   = false;

    public final boolean fullRobot;
    private final Pivot pivot;
    private final Drivetrain drivetrain;
    private final Roller roller;
    private final Shifter shifter;
    private final Pneumatics pneumatics;
    private final Climb climb;
    private final LEDs leds;
    private final GyroLance gyro;
    private final PoseEstimatorLance poseEstimator;
    private final CameraLL camera;

    private final CommandXboxController operatorController;
    private final CommandXboxController driverController;

    // private final SendableChooser<Command> autoChooser;


    RobotContainer() 
    {
        fullRobot           = (useFullRobot);
        pivot               = (useFullRobot || usePivot)                ? new Pivot()                                                                                                  : null;
        gyro                = (useFullRobot || useDrivetrain)           ? new GyroLance()                                                                                              : null;
        drivetrain          = (useFullRobot || useDrivetrain)           ? new Drivetrain(gyro)                                                                                         : null;
        roller              = (useFullRobot || useRoller)               ? new Roller()                                                                                                 : null;
        pneumatics          = (useFullRobot || usePneumatics)           ? new Pneumatics()                                                                                             : null;
        //FIXME shifter requires pneumatics doesn't it?
        shifter             = (useFullRobot || useShifter)              ? new Shifter()                                                                                                : null;
        leds                = (useFullRobot || useLEDs)                 ? new LEDs()                                                                                                   : null;
        climb               = (useFullRobot || useClimb)                ? new Climb()                                                                                                  : null;
        camera              = (useFullRobot || useCamera)               ? CameraLL.makeCamera(Constants.Camera.CAMERA)                                                                 : null;
        // FIXME camera maybe null
        poseEstimator       = (useFullRobot || usePoseEstimator)        ? new PoseEstimatorLance(gyro, drivetrain, camera)                                                             : null;
        driverController    = (useFullRobot || useDriverController)     ? new CommandXboxController(Constants.Controllers.DRIVER_CONTROLLER_PORT)                                      : null;
        operatorController  = (useFullRobot || useOperatorController)   ? new CommandXboxController(Constants.Controllers.OPERATOR_CONTROLLER_PORT)                                    : null;

        // Elastic = (useFullRobot || useElastic)   ? new Elastic(this) : null;

        // registerNamedCommands();

        // if(drivetrain != null)
        // {
        //     autoChooser = AutoBuilder.buildAutoChooser();
        //     SmartDashboard.putData("Auto Chooser", autoChooser);
        // }
        // else
        // {
        //     autoChooser = null;
        // }

        // configurePathPlannerLogging();
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

    public Pneumatics getPneumatics()
    {
        return pneumatics;
    }

    public Climb getClimb()
    {
        return climb;
    }

    public LEDs getLEDs()
    {
        return leds;
    }

    public CameraLL getCamera()
    {
        return camera;
    }

    public PoseEstimatorLance getPoseEstimator()
    {
        return poseEstimator;
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

    // public void registerNamedCommands()
    // {
    //     NamedCommands.registerCommand("Intake Algae", GeneralCommands.intakeAlgaeCommand());
    //     NamedCommands.registerCommand("Score Algae", GeneralCommands.scoreAlgaeCommand());
    //     NamedCommands.registerCommand("Score Coral", GeneralCommands.scoreAlgaeCommand());
    //     NamedCommands.registerCommand("LED Red", GeneralCommands.setLEDSolid(Color.kRed));
    //     NamedCommands.registerCommand("LED BlUE", GeneralCommands.setLEDSolid(Color.kBlue));
    // }


    // public Command getAutonomousCommand() 
    // {
    //     return autoChooser.getSelected();

    //     // return new PathPlannerAuto("TEST AUTO - MOVE FORWARD 2M");
    // }

    // private Field2d field; // object to put on dashboards
    // /**
    //  * Turn on all PathPlanner logging to a Field2d object for NT table "SmartDashboard".
    //  * <p>PP example from its documentation.
    //  */
    // private void configurePathPlannerLogging()
    // {
    //     field = new Field2d();
    //     SmartDashboard.putData("Field", field);

    //     // Logging callback for current robot pose
    //     PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
    //         // Do whatever you want with the pose here
    //         field.setRobotPose(pose);
    //     });

    //     // Logging callback for target robot pose
    //     PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
    //         // Do whatever you want with the pose here
    //         field.getObject("target pose").setPose(pose);
    //     });

    //     // Logging callback for the active path, this is sent as a list of poses
    //     PathPlannerLogging.setLogActivePathCallback((poses) -> {
    //         // Do whatever you want with the poses here
    //         field.getObject("path").setPoses(poses);
    //     });
    // }
}
