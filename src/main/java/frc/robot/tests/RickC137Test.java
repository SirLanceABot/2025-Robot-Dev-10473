//FIXME CHANGE PAthPlannerLance AutoBuilder to use drivetrain PID



package frc.robot.tests;

import java.lang.invoke.MethodHandles;

import frc.robot.RobotContainer;
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

    public RickC137Test(RobotContainer robotContainer)
    {
      System.out.println("  Constructor Started:  " + fullClassName);

      this.robotContainer = robotContainer;

      System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {
        // Activate Drivetrain PID Velocity Tuning
        robotContainer.getDrivetrain().new TuneVelocityPID().schedule();
    }

    @Override
    public void periodic() {
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public void exit() {
        throw new UnsupportedOperationException("Unimplemented method 'exit'");
    }
}
//     String toString(double[] array) {
//         return Arrays.stream(array)
//             .mapToObj(i -> String.format("%5.2f", i))
//            // .collect(Collectors.joining(", ", "[", "]"));
//             .collect(Collectors.joining("|", "|", "|"));
//   }

