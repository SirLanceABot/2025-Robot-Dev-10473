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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;

import frc.robot.RobotContainer;

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
    private final DoublePublisher voltageStepSizePublisher;
    private final DoublePublisher leftLeaderVelocityPublisher;
    private final DoublePublisher rightLeaderVelocityPublisher;
    private final DoublePublisher leftLeaderMotorVoltagePublisher;
    private final DoublePublisher rightLeaderMotorVoltagePublisher;

    // private TimestampedDouble data;
    private double voltageStepSize = 10.;


    public RickC137Test(RobotContainer robotContainer)
    {
      System.out.println("  Constructor Started:  " + fullClassName);

      this.robotContainer = robotContainer;

        var name = "sysid4237";
        var table = NetworkTableInstance.getDefault().getTable(name);
        voltageStepSizePublisher = table.getDoubleTopic("voltageStepSize").publish();
        leftLeaderVelocityPublisher = table.getDoubleTopic("leftLeaderVelocity").publish();
        rightLeaderVelocityPublisher = table.getDoubleTopic("rightLeaderVelocity").publish();
        leftLeaderMotorVoltagePublisher = table.getDoubleTopic("leftLeaderMotorVoltage").publish();
        rightLeaderMotorVoltagePublisher = table.getDoubleTopic("rightLeaderMotorVoltage").publish();
        
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {
        // Activate Drivetrain PID Velocity Tuning
        robotContainer.getDrivetrain().new TuneVelocityPID().schedule();
        //FIXME take voltage step here
        // run a few seconds??
    }

    @Override
    public void periodic() {
        leftLeaderVelocityPublisher.set(robotContainer.getDrivetrain().getLeftLeaderVelocity());
        rightLeaderVelocityPublisher.set(robotContainer.getDrivetrain().getRightLeaderVelocity());
        leftLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getLeftLeaderMotorVoltage());
        rightLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getRightLeaderMotorVoltage());
        voltageStepSizePublisher.set(voltageStepSize);
    }

    @Override
    public void exit() {
        robotContainer.getDrivetrain().stopDrive();
    }
}
//     String toString(double[] array) {
//         return Arrays.stream(array)
//             .mapToObj(i -> String.format("%5.2f", i))
//            // .collect(Collectors.joining(", ", "[", "]"));
//             .collect(Collectors.joining("|", "|", "|"));
//   }

