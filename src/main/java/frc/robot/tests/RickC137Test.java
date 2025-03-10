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
import edu.wpi.first.wpilibj.Timer;
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
    private final DoublePublisher leftLeaderAccelerationPublisher;
    private final DoublePublisher rightLeaderAccelerationPublisher;
    private final DoublePublisher leftLeaderMotorVoltagePublisher;
    private final DoublePublisher rightLeaderMotorVoltagePublisher;

    // private TimestampedDouble data;
    private double stepSize = 1.; // %VBus -1 to +1


    public RickC137Test(RobotContainer robotContainer)
    {
      System.out.println("  Constructor Started:  " + fullClassName);

      this.robotContainer = robotContainer;

        var name = "sysid4237";
        var table = NetworkTableInstance.getDefault().getTable(name);
        voltageStepSizePublisher = table.getDoubleTopic("voltageStepSize").publish();
        leftLeaderAccelerationPublisher = table.getDoubleTopic("leftLeaderAcceleration").publish();
        rightLeaderAccelerationPublisher = table.getDoubleTopic("rightLeaderAcceleration").publish();
        leftLeaderMotorVoltagePublisher = table.getDoubleTopic("leftLeaderMotorVoltage").publish();
        rightLeaderMotorVoltagePublisher = table.getDoubleTopic("rightLeaderMotorVoltage").publish();
        
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private double leftLeaderVelocityPrevious;
    private double rightLeaderVelocityPrevious;
    private double timePrevious;

    @Override
    public void init() {
//FIXME don't activate both at the same time!!!!!!!!

        // Activate Drivetrain PID Velocity Tuning
        robotContainer.getDrivetrain().new TuneVelocityPID().schedule();

        //Activate acceleration measurement //FIXME change to a command
        leftLeaderVelocityPrevious = robotContainer.getDrivetrain().getLeftLeaderVelocity();
        rightLeaderVelocityPrevious = robotContainer.getDrivetrain().getRightLeaderVelocity();
        timePrevious = Timer.getFPGATimestamp();
        robotContainer.getDrivetrain().setDrive(stepSize);
        // run a few seconds??
    }

    @Override
    public void periodic() {
        var time = Timer.getFPGATimestamp();
        var leftLeaderVelocity = robotContainer.getDrivetrain().getLeftLeaderVelocity();
        var rightLeaderVelocity = robotContainer.getDrivetrain().getRightLeaderVelocity();
        var leftAcceleration =
            (leftLeaderVelocity - leftLeaderVelocityPrevious)
            / (time - timePrevious);
        var rightAcceleration =
            (rightLeaderVelocity - rightLeaderVelocityPrevious)
            / (time - timePrevious);
        leftLeaderVelocityPrevious = leftLeaderVelocity;
        rightLeaderVelocityPrevious = rightLeaderVelocity;
        timePrevious = time;
        
        leftLeaderAccelerationPublisher.set(leftAcceleration);
        rightLeaderAccelerationPublisher.set(rightAcceleration);
        leftLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getLeftLeaderMotorVoltage());
        rightLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getRightLeaderMotorVoltage());
        voltageStepSizePublisher.set(stepSize);
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

