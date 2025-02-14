package frc.robot.sensors;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Implementation of the camera class for Limelights
 * <p> This class acquires the robot pose in field from a Limelight MegaTag2
 * <p>Typical usage:
 * 
 <pre>

    // construct the LL1 object once
    CameraLL LL1;
    boolean useLL1 = true;

    LL1 = useLL1 ? CameraLL.makeCameraLL("limelight") : null; // make a limelight - null if it doesn't exist or not requested

    // set the camera stream mode as often as necessary
    if (LL1 != null)
    {
      // example of two ways to specify the view of the external camera (method overloads)
      LL1.setStreamMode_PiPSecondary();
      // or
      LL1.setStreamMode(CameraLL.StreamMode.External);
    }

    // periodically set and get data
    if (LL1 != null)
    {
      // for MegaTag2 set the robot orientation from the gyro heading and rate.
      //FIXME get the gyro values somehow but here are zeros for test data - limits what AprilTags make sense
      LL1.setRobotOrientation(0., 0., 0., 0., 0., 0.);

      // example use of interpreting the JSON string from LL - it's a bit slow; not implemented here yet unless wanted
      LimelightHelpers.getLatestResults("limelight"); // 0.2 milliseconds (1 tag) to 0.3 milliseconds (2 tags), roughly

      LL1.update();

      // some methods we might use
      if (LL1.isFresh())
      {
          LL1.publishPose3d();

          System.out.println(LL1.getPoseTimestampSeconds() + " Time of the pose");
          System.out.println(LL1.getPose2d() + " MegaTag2 pose");
          System.out.println(LL1.getPose3d() + " MegaTag2 pose");
          System.out.println(LL1.getTX() + " tx");
          System.out.println(LL1.getTXNC() + " txnc");
          System.out.println(LL1.getTY() + " ty");
          System.out.println(LL1.getTYNC() + " tync");
          System.out.println(LL1.getLatency() + " total latency of the pose");
          System.out.println(LL1.getTagCount() + " number of tags seen to make the pose");
          System.out.println(LL1.getTagSpan() + " span of the tags");
          System.out.println(LL1.getAvgTagDist() + " average distance from tags to robot");
          System.out.println(LL1.getAvgTagArea() + " average area of the tags in their frames\n");

          // 23.088082194656373 Time of the pose
          // Pose2d(Translation2d(X: 8.77, Y: 4.03), Rotation2d(Rads: 0.00, Deg: 0.00)) MegaTag2 pose      
          // Pose3d(Translation3d(X: 8.77, Y: 4.03, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))) MegaTag2 pose
          // 5.994534015655518 tx
          // 7.574864387512207 txnc
          // 5.370254039764404 ty
          // 4.8771843910217285 tync
          // 28.33680534362793 total latency of the pose
          // 1 number of tags seen to make the pose
          // 0.0 span of the tags
          // 0.535089273470325 average distance from tags to robot
          // 8.579281717538834 average area of the tags in their frames
      }
      LL1.setStreamMode_Standard();
      LL1.setStreamMode_PiPMain();
      LL1.setStreamMode_PiPSecondary();
      LL1.setStreamMode(streamMode);

      // data usage for pose estimation
      // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); //FIXME need stddev tuning/filtering
      // m_poseEstimator.addVisionMeasurement(LL1.getPose2d(), LL1.getPoseTime());
      }
    }
</pre>
 */
public class CameraLL extends CameraLance {

    private final String name;
    private boolean isFresh;

    // fields associated with Megatag2 blue pose
    private Pose2d pose;
    private double timestampSeconds;
    private double latency;
    private int tagCount;
    private double tagSpan;
    private double avgTagDist;
    private double avgTagArea;
    private Pose3d pose3d = new Pose3d(); // initialize in case it's used before being set (not supposed to happen)

    private long previousTimestamp = 0; // arbitrary initial time to start
    
    private DoubleSubscriber tx;
    private DoubleSubscriber ty;
    private DoubleSubscriber txnc;
    private DoubleSubscriber tync;
    private final DoubleArraySubscriber t2d;
    private final DoubleArraySubscriber botpose_orb_wpiblue;
    // private final DoubleArraySubscriber stddevs;
    private final DoubleArrayPublisher robot_orientation_set;
    private final DoublePublisher stream;


    private CameraLL(String name) {
        super(name);
        if (!isAvailable(name))
        {
          throw new RuntimeException("Limelight named " + name + " is not available");
        }

        this.name = name;

        // Get the limelight table
        var table = CameraLance.NTinstance.getTable(name);

        /*
        tx
        	double	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
         */
        tx = table.getDoubleTopic("tx").subscribe(Double.MAX_VALUE); // default is unrealistically big

        /*
        ty
        	double	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
         */
        ty = table.getDoubleTopic("ty").subscribe(Double.MAX_VALUE);


        /*
        txnc
        	double	Horizontal Offset From Principal Pixel To Target (degrees)
         */
        txnc = table.getDoubleTopic("txnc").subscribe(Double.MAX_VALUE); // default is unrealistically big


        /*
        tync
        	double	Vertical Offset From Principal Pixel To Target (degrees)
         */
        tync = table.getDoubleTopic("tync").subscribe(Double.MAX_VALUE);
        
        /*
        t2d
        	doubleArray containing several values for matched-timestamp statistics:
                targetValid, [0]
                targetCount, [1]
                targetLatency, [2]
                captureLatency, [3]
                tx, [4]
                ty, [5]
                txnc, [6]
                tync, [7]
                ta, [8]
                tid, [9]
                targetClassIndexDetector, [10]
                targetClassIndexClassifier, [11]
                targetLongSidePixels, [12]
                targetShortSidePixels, [13]
                targetHorizontalExtentPixels, [14]
                targetVerticalExtentPixels, [15]
                targetSkewDegrees [16]
         */
        t2d = table.getDoubleArrayTopic("t2d").subscribe(new double[]{}); // default is no data (array length = 0)

        /*
        botpose_orb_wpiblue
        	doubleArray
                Robot transform in field-space (Megatag2 blue driverstation WPILIB origin).
                Translation (X,Y,Z) in meters, [0-2]
                Rotation(Roll,Pitch,Yaw) in degrees, [3-5]
                total latency (cl+tl), [6]
                tag count, [7]
                tag span, [8]
                average tag distance from camera, [9]
                average tag area (percentage of image) [10]
         */
        botpose_orb_wpiblue = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{}); // default is no data (array length = 0)

        /*
        stddevs
            doubleArray MegaTag Standard Deviations
                MT1x,
                MT1y,
                MT1z,
                MT1roll,
                MT1pitch,
                MT1Yaw,
                MT2x,
                MT2y,
                MT2z,
                MT2roll,
                MT2pitch,
                MT2yaw
         */
        // stddevs = table.getDoubleArrayTopic("stddevs").subscribe(
        //     new double[]{ // default stddevs huge number so as not to be used but validation should have prevented that anyway
        //         9999., 9999., 9999., 9999., 9999., 9999.,
        //         9999., 9999., 9999., 9999., 9999., 9999.
        //     });

        /*
        robot_orientation_set
        	doubleArray
                SET Robot Orientation and angular velocities in degrees and degrees per second[yaw, yawrate, pitch, pitchrate, roll, rollrate]
         */
        robot_orientation_set = table.getDoubleArrayTopic("robot_orientation_set").publish();

        /*
        stream
            	Sets limelight's streaming mode
                    0	Standard - Side-by-side streams if a webcam is attached to Limelight
                    1	PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
                    2	PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        stream = table.getDoubleTopic("stream").publish();
    }

    /**
     * Essentially the constructor for a limelight cameraserver.java
     * @param name set in the limelight
     * @return a limelight object or null if it doesn't exist
     */
    public static CameraLL makeCameraLL(String name)
    {
        if (!isAvailable(name))
        {
            return null;
        }

        return new CameraLL(name);
    }

    /**
     * Enables standard viewing of cameras mode. If both internal and external cameras available they are viewed side-by-side.
     */
    public void setStreamMode_Standard()
    {
        stream.set(0.);
    }

    /**
     * Enables Picture-in-Picture mode viewing internal camera with external stream in the corner.
     */
    public void setStreamMode_PiPMain()
    {
        stream.set(1.);
    }

    /**
     * Enables Picture-in-Picture mode viewing external camera with primary camera in the corner.
     */
    public void setStreamMode_PiPSecondary()
    {
        stream.set(2.);
    }

    public enum StreamMode {Both, External, Internal}
    public void setStreamMode(StreamMode streamMode)
    {
        stream.set((double)streamMode.ordinal());
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees 
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate)
    {
        double[] gyro = new double[6];
        gyro[0] = yaw;
        gyro[1] = yawRate;
        gyro[2] = pitch;
        gyro[3] = pitchRate;
        gyro[4] = roll;
        gyro[5] = rollRate;
        robot_orientation_set.set(gyro); // robot orientation sent to LL for MegaTag2
        CameraLance.NTinstance.flush();
    }

    /**
     * getter for validity of last acquisition
     * <p>data must be valid and different timestamp than the previous data
     * 
     * @return freshness of the last acquisition
     */
    public boolean isFresh()
    {
      return isFresh;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @return Horizontal offset angle in degrees
     */
    public double getTX() {
      return tx.get();
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   * @return Vertical offset angle in degrees
   */
  public double getTY() {
      return ty.get();
  }

  /**
   * Gets the horizontal offset from the principal pixel/point to the target in degrees.
   * <p>This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
   * @return Horizontal offset angle in degrees
   */
  public double getTXNC() {
      return txnc.get();
  }

  /**
   * Gets the vertical offset from the principal pixel/point to the target in degrees.
   * <p>This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
   * @return Vertical offset angle in degrees
   */
  public double getTYNC() {
      return tync.get();
  }

    /**
     * MegaTag2 blue pose getter from pose for PoseEstimator.addVisionMeasurement
     * @return
     */
    public Pose2d getPose2d()
    {
      return pose;
    }

    /**
     * MegaTag2 blue pose getter from pose
     * @return
     */
    public Pose3d getPose3d()
    {
      return pose3d;
    }

    /**
     * Timestamp getter from pose for PoseEstimator.addVisionMeasurement
     * @return
     */
    public double getPoseTimestampSeconds()
    {
        return timestampSeconds;
    }

    /**
     * Total latency getter from pose
     * @return
     */
    public double getLatency()
    {
        return latency;
    }

    /**
     * Tag count getter from pose
     * @return
     */
    public int getTagCount()
    {
        return tagCount;
    }


    /**
     * Tag span getter from pose
     * @return
     */
    public double getTagSpan()
    {
        return tagSpan;
    }


    /**
     * Average tag distance getter from pose
     * @return
     */
    public double getAvgTagDist()
    {
        return avgTagDist;
    }


    /**
     * Average tag area getter from pose
     * @return
     */
    public double getAvgTagArea()
    {
        return avgTagArea;
    }

// average distance " + pose.value[9]
// pose.tagCount);
//         System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan);
//         System.out.printf("Average Tag Distance: %.2f meters%n", pose.avgTagDist);
//         System.out.printf("Average Tag Area: %.2f%% of image%n", pose.avgTagArea);

    /**
     * Read the latest Limelight values.
     * <p>Call this method in the robot iterative loop.
     */
    public void update() {

        // get LL data needed to determine validity
        var stats = t2d.getAtomic();

        // check if new data
        isFresh = stats.timestamp == previousTimestamp && 17 == stats.value.length && 1.0 == stats.value[0];
        previousTimestamp = stats.timestamp;

        if (isFresh())
        {
            var poseTemp = botpose_orb_wpiblue.getAtomic(); // get the LL MegaTag2 pose data

            pose3d = new Pose3d(poseTemp.value[0], poseTemp.value[1], poseTemp.value[2], new Rotation3d(poseTemp.value[3], poseTemp.value[4], poseTemp.value[5])); // robot in field 3d pose
            pose = new Pose2d(poseTemp.value[0], poseTemp.value[1], new Rotation2d(Units.degreesToRadians(poseTemp.value[5]))); // robot in field 2d pose
            var timestampSecondsTemp = poseTemp.timestamp;
            latency = poseTemp.value[6];
            timestampSeconds = (timestampSecondsTemp / 1000000.0) - (latency / 1000.0); // Convert server timestamp from microseconds to seconds and adjust for latency
            tagCount = (int)poseTemp.value[7];
            tagSpan = poseTemp.value[8];
            avgTagDist = poseTemp.value[9];
            avgTagArea = poseTemp.value[10];
        }
        else
        if (stats.value.length == 0)
        {
            System.out.println(name + " not connected");
        }   
    }

    public String toString()
    {
      StringBuilder sb = new StringBuilder(500);

            // sb.append("local time " + stats.timestamp + ", server time " + stats.serverTime
            // + ", valid " + stats.value[0] + ", count " + stats.value[1] + " total latency " + (stats.value[2]+stats.value[3])
            // + ", horizontal offset " + stats.value[4] + " tag id " + stats.value[9] + " ,skew deg " +stats.value[16]);

            // sb.append("addVisionMeasurement: Pose2d " + pose + ", timestamp " + timestampSeconds);
            // sb.append(pose3d.toString());
            // sb.append("total latency " + pose.value[6] + ", count " + pose.value[7] + ", span "
            //  + pose.value[8] + ", average distance " + pose.value[9] + ", average area " + pose.value[10]);


// addVisionMeasurement Pose2d Pose2d(Translation2d(X: 11.81, Y: 4.04), Rotation2d(Rads: 0.00, Deg: 0.00)), timestamp 97.64429015905762
// local time 171282476, server time 171282476, valid 1.0, count 2.0 total latency 174.8796157836914, horizontal offset 10.477392196655273 tag id 12.0 ,skew deg 75.9100570678711
// Pose3d(Translation3d(X: 1.26, Y: -0.07, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))
// total latency 174.87960815429688, count 2.0, span 0.335460891519132, average distance 0.7605234507148955, average area 4.441176541149616

            // var sd = stddevs.get();
            // System.out.println("standard deviations - x, y, z, roll, pitch, yaw");
            // System.out.format("MegaTag1 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[0], sd[1], sd[2], sd[3], sd[4], sd[5]);
            // System.out.format("MegaTag2 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[6], sd[7], sd[8], sd[9], sd[10], sd[11]);

// standard deviations - x, y, z, roll, pitch, yaw
// MegaTag1   3.48,   0.56,   0.53,  18.04,   8.94,  66.13
// MegaTag2   0.12,   0.03,   0.00,   0.00,   0.00,   0.00
        return sb.toString();
    }

  /**
   * Verify limelight name exists as a table in NT.
   * <p>
   * This check is expected to be run once during robot construction and is not intended to be checked
   * in the iterative loop.
   *
   * @param limelightName Limelight Name to check for table existence.
   * @return true if an NT table exists with requested LL name.
   * <p>false and issues a WPILib Error Alert if requested LL doesn't appear as an NT table.
   */
  @SuppressWarnings("resource")
  public static boolean isAvailable(String limelightName)
  {
    // LL sends key "getpipe" if it's on so check that
    // put in a delay if needed to help assure NT has latched onto the LL if it is transmitting
    for (int i = 1; i <= 15; i++)
    {
      if (CameraLance.NTinstance.getTable(limelightName).containsKey("getpipe"))
      {
        return true;
      }
      System.out.println("waiting " + i + " of 15 seconds for limelight named " + limelightName + " to attach");
      try
      {
        Thread.sleep((long) Seconds.of(1).in(Milliseconds));
      } catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    }
    String errMsg = "Your limelight name \"" + limelightName +
                    "\" is invalid; doesn't exist on the network (no getpipe key).\n" +
                    "These may be available:" +
                    CameraLance.NTinstance.getTable("/").getSubTables().stream()
                                        .filter(ntName -> ((String) (ntName)).startsWith("limelight"))
                                        .collect(Collectors.joining("\n")) +
                                        "If in simulation, check LL Dashboard: Settings / Custom NT Server IP:";
    new Alert(errMsg, AlertType.kError).set(true);
    return false;
  }
}
