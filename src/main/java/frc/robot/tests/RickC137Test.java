// activate this test class in the TestMode class


//FIXME CHANGE PathPlannerLance AutoBuilder to use drivetrain PID

// B-Bot drivetrain testing rkt 3/9/2025 LEF Learning Center carpet
// TalonFX velocity mode
// test velocity 1 m/s
// Ks = 0.13
// Kv = 3.15
// Kp = 15. [seems a little better than 10 or 20]

package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import frc.robot.RobotContainer;

public class RickC137Test implements Test {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private final RobotContainer robotContainer;

    public RickC137Test(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;
        
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {}

    @Override
    public void periodic() {}

    @Override
    public void exit() {}
}
//     String toString(double[] array) {
//         return Arrays.stream(array)
//             .mapToObj(i -> String.format("%5.2f", i))
//            // .collect(Collectors.joining(", ", "[", "]"));
//             .collect(Collectors.joining("|", "|", "|"));
//   }

// private TimestampedDouble data;