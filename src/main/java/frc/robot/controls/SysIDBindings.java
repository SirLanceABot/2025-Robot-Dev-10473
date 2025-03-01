package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SysIDBindings 
{
    private static CommandXboxController controller;
    private static Drivetrain drivetrain;

    private SysIDBindings()
    {}

    public static void createBindings(RobotContainer robotContainer)
    {
        controller = robotContainer.getSysIDController();
        drivetrain = robotContainer.getDrivetrain();
        if(controller != null)
        {
            configSysid();
        }
    }
        /**
     * SysId driving runs with driver controller
     * 
     * <p>Press and hold the left bumper and one other button A, B, X, Y.
     * <p>Release buttons to immediately halt the test drives.
     * <p>left bumper + A drives slowly forward
     * <p>left bumper + B drives slowly backward
     * <p>left bumper + X drives fast forward
     * <p>left bumper + Y drives fast backward
     * <p>Suggested sequence:
     * <p>slow forward and stop/pause
     * <p>slow backward and stop/pause
     * <p>fast forward and stop/pause
     * <p>fast backward and stop
     */
    private static void configSysid()
    {
        if(drivetrain != null)
        {
            Trigger aButtonTriggerSysid = controller.a().and(controller.leftBumper());
            aButtonTriggerSysid.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

            Trigger bButtonTriggerSysid = controller.b().and(controller.leftBumper());
            bButtonTriggerSysid.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

            Trigger xButtonTriggerSysid = controller.x().and(controller.leftBumper());
            xButtonTriggerSysid.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));

            Trigger yButtonTriggerSysid = controller.y().and(controller.leftBumper());
            yButtonTriggerSysid.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
    }
}
