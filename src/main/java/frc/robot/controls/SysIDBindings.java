package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

//FIXME It is expected that the use of this class is commented out in Robot.java
// and only activated purposely by uncommenting the code in Robot.java:
// SysIDBindings.createBindings(robotContainer);

/**
 * Commands to identify the drivetrain
 * WPILib SYSID
 * Tune velocity PIDF
 * Measure Acceleration
 */
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
        if(controller != null && drivetrain != null)
        {
            configSysid();
        }
    }

    private static void configSysid()
    {
        /**
         * WPILib SYSID runs
         * 
         * Press and hold the left bumper and one other button A, B, X, Y.
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
        Trigger aButtonTriggerSysid = controller.a().and(controller.leftBumper());
        aButtonTriggerSysid.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        Trigger bButtonTriggerSysid = controller.b().and(controller.leftBumper());
        bButtonTriggerSysid.whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        Trigger xButtonTriggerSysid = controller.x().and(controller.leftBumper());
        xButtonTriggerSysid.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));

        Trigger yButtonTriggerSysid = controller.y().and(controller.leftBumper());
        yButtonTriggerSysid.whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        /**
         * Run acceleration Measurement in LOW GEAR that must be preset by the driver before running this test.
         * <p>Press and hold the left bumper and right bumper
         * <p>Releasing either bumper will stop the robot and data collection
         * <p>Output logged to NT ASTable.
         */
        Trigger accelerationMeasurement = controller.rightBumper().and(controller.leftBumper());
        accelerationMeasurement.whileTrue(
            drivetrain.new MeasureAcceleration()
            .withTimeout(2.5)); // emergency off - in low gear this is could be several feet

        /**
         * Tune drivetrain PIDF in LOW GEAR that must be preset by the driver before running this test.
         * <p>Press and hold left bumper then press BACK button. They then may be released.
         * <p>Interactive tuning shows in the SmartDashboard.
         */
        Trigger drivetrainTunePID = controller.back().and(controller.leftBumper());
        drivetrainTunePID.onTrue(
            drivetrain.new TuneVelocityPID());
    }
}
