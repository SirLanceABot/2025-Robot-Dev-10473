// activate this test class in the TestMode class


//FIXME CHANGE PathPlannerLance AutoBuilder to use drivetrain PID

// B-Bot drivetrain testing rkt 3/9/2025 LEF Learning Center carpet
// TalonFX velocity mode
// test velocity 1 m/s
// Ks = 0.14 [no more than this!]
// Kv = 3.15
// Kp = 15. [seems a little better than 10 or 20]

package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.TimestampedDouble;


import frc.robot.RobotContainer;
import frc.robot.sensors.CameraLance;
import frc.robot.subsystems.Drivetrain.TuneVelocityPID;

public class RickC137Test implements Test {
      // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // *** CLASS & INSTANCE VARIABLES ***
    // Put all class and instance variables here.
    private final RobotContainer robotContainer;
    // private final DoublePublisher stream;
    // private TimestampedDouble data;


    public RickC137Test(RobotContainer robotContainer)
    {
      System.out.println("  Constructor Started:  " + fullClassName);

      this.robotContainer = robotContainer;

        // var name = "sysid4237";
        // var table = CameraLance.NTinstance.getTable(name);
        // stream = table.getDoubleTopic("stream").publish();
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {
        // Activate Drivetrain PID Velocity Tuning
        robotContainer.getDrivetrain().new TuneVelocityPID().schedule();

    }

    @Override
    public void periodic() {
        // stream.set(1.);
    }

    @Override
    public void exit() {}
}
//     String toString(double[] array) {
//         return Arrays.stream(array)
//             .mapToObj(i -> String.format("%5.2f", i))
//            // .collect(Collectors.joining(", ", "[", "]"));
//             .collect(Collectors.joining("|", "|", "|"));
//   }

