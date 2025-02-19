package frc.robot.tests;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RThomasTest implements Test {

    void testor()
    {
        PathPlannerPath path = null;
        try {
            path = PathPlannerPath.fromPathFile("TEST - Move Forward 2M");
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }

        PathPlannerLogging.logActivePath(path);
    }


/*
 * PathPlannerLib provides the ability to set custom logging callbacks that will be called when the built-in path following commands
 *  are running. You can set these callbacks through the PathPlannerLogging class.

These can be used for logging path following data with logging frameworks such as AdvantageKit, or visualization with a Field2d Widget.

static void logActivePath(PathPlannerPath path) Log the active path.
static void logCurrentPose(Pose2d pose)Log the current robot pose.
static void logTargetPose(Pose2d targetPose) Log the target robot pose.
static void setLogActivePathCallback(Consumer<List<Pose2d>> logActivePath) Set the logging callback for the active path
static void setLogCurrentPoseCallback(Consumer<Pose2d> logCurrentPose) Set the logging callback for the current robot pose
static void setLogTargetPoseCallback(Consumer<Pose2d> logTargetPose) Set the logging callback for the target robot pose
 */
   
    private Field2d field;

    public void RobotContainer(){
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);

            // PathPlannerLogging.logCurrentPose(pose);

        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);

            // PathPlannerLogging.logTargetPose(pose);

        });

        // Logging callback for the active path, this is sent as a list of poses that comprise the path
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);

            // PathPlannerLogging.logActivePath(poses) has the same set of poses comprising the active path
        });
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public void exit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'exit'");
    }
}
//     String toString(double[] array) {
//         return Arrays.stream(array)
//             .mapToObj(i -> String.format("%5.2f", i))
//            // .collect(Collectors.joining(", ", "[", "]"));
//             .collect(Collectors.joining("|", "|", "|"));
//   }