// activate this test class in the TestMode class

// B-Bot drivetrain testing rkt 3/9/2025 LEF Learning Center carpet
// TalonFX velocity mode
// test velocity 1 m/s
// Ks = 0.13
// Kv = 3.15
// Kp = 15. [seems a little better than 10 or 20]

package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import frc.robot.RobotContainer;

public class DrivetrainTuning implements Test {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final RobotContainer robotContainer;
    private boolean tunePID = false;
    private boolean measureAcceleration = true;

    public DrivetrainTuning(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {

        // one or the other but not both
        if(tunePID) // Activate Drivetrain PID Velocity Tuning
        {
            robotContainer.getDrivetrain().new TuneVelocityPID().schedule();
        }
        else
        if(measureAcceleration) // Activate acceleration measurement
        {
            robotContainer.getDrivetrain().new MeasureAcceleration().schedule();       
        }
    }

    @Override
    public void periodic() {}

    @Override
    public void exit() {
        robotContainer.getDrivetrain().stopDrive();
    }
}
