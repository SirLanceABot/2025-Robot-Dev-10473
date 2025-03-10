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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RickC137Test implements Test {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    private boolean tunePID = false; // one or the other but not both
    private boolean measureAcceleration = true && !tunePID; // one or the other but not both
    
    private final double stepSize = 1.; // %VBus -1 to 0 to +1
    private final double stopAt = 0.2; // stop motors if below this m/s/s
    private double leftLeaderVelocityPrevious;
    private double rightLeaderVelocityPrevious;
    private double timePrevious;
    private boolean firstTime;
    private final RobotContainer robotContainer;
    private final DoublePublisher voltageStepSizePublisher;
    private final DoublePublisher timePublisher;
    private final DoublePublisher leftLeaderAccelerationPublisher;
    private final DoublePublisher rightLeaderAccelerationPublisher;
    private final DoublePublisher leftLeaderMotorVoltagePublisher;
    private final DoublePublisher rightLeaderMotorVoltagePublisher;

    public RickC137Test(RobotContainer robotContainer)
    {
        System.out.println("  Constructor Started:  " + fullClassName);

        this.robotContainer = robotContainer;

        var table = NetworkTableInstance.getDefault().getTable(Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        voltageStepSizePublisher = table.getDoubleTopic("voltageStepSize").publish();
        timePublisher = table.getDoubleTopic("time").publish();
        leftLeaderAccelerationPublisher = table.getDoubleTopic("leftLeaderAcceleration").publish();
        rightLeaderAccelerationPublisher = table.getDoubleTopic("rightLeaderAcceleration").publish();
        leftLeaderMotorVoltagePublisher = table.getDoubleTopic("leftLeaderMotorVoltage").publish();
        rightLeaderMotorVoltagePublisher = table.getDoubleTopic("rightLeaderMotorVoltage").publish();
        
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    @Override
    public void init() {

        // Activate Drivetrain PID Velocity Tuning
        if(tunePID)
        {
            robotContainer.getDrivetrain().new TuneVelocityPID().schedule();
        }

        // Activate acceleration measurement
        //FIXME change to a command
        if(measureAcceleration)
        {
            firstTime = true;            
        }
    }

    @Override
    public void periodic() {

        if(measureAcceleration)
        {
            robotContainer.getDrivetrain().setDrive(stepSize); // first time required, then refresh every time

            if(firstTime)
            {
                // assume starting at rest at time "0"
                timePrevious = Timer.getFPGATimestamp();
                leftLeaderVelocityPrevious = 0.;
                rightLeaderVelocityPrevious = 0.;
                firstTime = false;
            }
            else
            {
                var time = Timer.getFPGATimestamp();

                var leftLeaderVelocity = robotContainer.getDrivetrain().getLeftLeaderVelocity();
                var rightLeaderVelocity = robotContainer.getDrivetrain().getRightLeaderVelocity();

                var leftAcceleration =
                    (leftLeaderVelocity - leftLeaderVelocityPrevious)
                    / (time - timePrevious);
                var rightAcceleration =
                    (rightLeaderVelocity - rightLeaderVelocityPrevious)
                    / (time - timePrevious);

                timePrevious = time;
                leftLeaderVelocityPrevious = leftLeaderVelocity;
                rightLeaderVelocityPrevious = rightLeaderVelocity;
                
                voltageStepSizePublisher.set(stepSize);
                timePublisher.set(time);
                leftLeaderAccelerationPublisher.set(leftAcceleration);
                rightLeaderAccelerationPublisher.set(rightAcceleration);
                leftLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getLeftLeaderMotorVoltage());
                rightLeaderMotorVoltagePublisher.set(robotContainer.getDrivetrain().getRightLeaderMotorVoltage());

                if(leftAcceleration < stopAt || rightAcceleration < stopAt)
                {
                    robotContainer.getDrivetrain().stopDrive();
                    measureAcceleration = false;
                }
            }
        }
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

    // private TimestampedDouble data;