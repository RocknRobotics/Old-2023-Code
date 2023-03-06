import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import java.util.*;
import java.io.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.QuinticHermiteSpline;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
TODO:
Get the robot radius
Get the numbers for the actuator/potentiometer
Spend time crunching numbers for positions
*/

public class laptopMain {
    static ArrayList<VideoCapture> captures = new ArrayList<VideoCapture>();
    static ArrayList<Double> widths = new ArrayList<Double>();
    static ArrayList<Double> heights = new ArrayList<Double>();
    static ArrayList<Double> perPixelAngleX = new ArrayList<Double>();
    static ArrayList<Double> cameraAngles = new ArrayList<Double>();
    static ArrayList<Double> cameraX = new ArrayList<Double>();
    static ArrayList<Double> cameraY = new ArrayList<Double>();
    static Mat img;

    //static final TrajectoryConfig myConfig = new TrajectoryConfig(Integer.MAX_VALUE, Integer.MAX_VALUE);
    //NEED radius to middle wheel in metres!!!
    static final double robotRadius = 0.5;
    //The length of the robot measured between the two bumpers that are perpendicular to the wheels
    static final double robotLength = 0.0;
    //The width of the robot measured between the two bumpers parallel to the wheels
    static final double robotWidth = 0.0;
    //Change if needed
    static final boolean blueAlliance = true;
    //Where the robot needs to be to be clear of the barrier (metres)
    static final double clearBarrierPosition = 0.0;
    //Field length (x)
    static final double fieldLength = Units.inchesToMeters(652.63);
    //Field width(y)
    static final double fieldWidth = 8.0137;
    //The imaginary line in the field where the robot should go to so that it just barely lines up with the corner station
    static final double cornerLineY = 0.0;
    //Line for lining up x in front of station
    static final double stationLine = 0.0;
    //Station y position tolerance
    static final double stationYTolerance = 0.0;
    //Station y positions (different from april tags since those are only like every other station)
    static final double[] stationYs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //Y position of the shelf
    static final double shelfY = 0.0;
    //X position of the shelf
    static final double shelfX = 0.0;

    static Thread positionThread;
    //Accel isn't actually important in this class, just for the dashboard
    static double accelX = 0.0;
    static double accelY = 0.0;
    static double velocityX = 0.0;
    static double velocityY = 0.0;
    static double positionX = 0.0;
    static double positionY = 0.0;

    static double angleAccel = 0.0;
    static double angleVelocity = 0.0;
    static double angle = blueAlliance ? 180.0 : 0.0;

    static double currTime = SmartDashboard.getNumber("FPGA Time", 0.0);
    static double prevVelocityX = 0.0;
    static double prevVelocityY = 0.0;
    static double prevAngularVelocity = 0.0;

    static Thread visionThread;

    static AprilTagDetector myDetector = new AprilTagDetector();
    static AprilTagDetection[] myTags;

    static AprilTagFieldLayout theLayout;
    static List<AprilTag> theTags;

    static double leftSpeed = 0.0;
    static double rightSpeed = 0.0;
    static double targetLeftSpeed = 0.0;
    static double targetRightSpeed = 0.0;
    static double targetAngle = 0.0;

    static Thread decider;

    public static void main(String[] args) {
        System.loadLibrary("ntcore");
        System.loadLibrary("ntcorejni");
        System.loadLibrary("apriltag");
        System.loadLibrary("apriltagjni");
        System.loadLibrary("wpiHal");
        System.loadLibrary("wpiHaljni");
        System.loadLibrary("wpiutil");
        System.loadLibrary("wpiutiljni");
        System.loadLibrary("opencv_java460");
        SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());

        SmartDashboard.putBoolean("Left Teeter", false);
        SmartDashboard.putBoolean("Middle Teeter", false);
        SmartDashboard.putBoolean("Right Teeter", false);
        
        myDetector.addFamily("tag16h5");

        try {
            theLayout = new AprilTagFieldLayout("fieldLayout.json");
            theTags = theLayout.getTags();
        } catch(IOException e) {
          System.out.println("File");
        }

        //Creates pre-built trajectories to avoid having to recalculate position every time if possible
        double shelfX = SmartDashboard.getBoolean("Blue Alliance", true) ? 40.0 : 0.0;
        double shelfY = fieldWidth - (34.43 - (robotWidth / 2.0));
        double stationX = Units.inchesToMeters(54.25) + (robotLength / 2.0) + (SmartDashboard.getBoolean("Blue Alliance", true) ? 0 : -1 * fieldLength);
        double[] stationY = new double[9];

        for(int i = 0; i < 9; i++) {
          switch(i) {
            case 0:
              stationY[i] = Units.inchesToMeters(20.19);
              break;
            case 1:
              stationY[i] = Units.inchesToMeters(42.19);
              break;
            case 2:
              stationY[i] = Units.inchesToMeters(64.19);
              break;
            case 3:
              stationY[i] = Units.inchesToMeters(86.19);
              break;
            case 4:
              stationY[i] = Units.inchesToMeters(108.19);
              break;
            case 5:
              stationY[i] = Units.inchesToMeters(130.19);
              break;
            case 6:
              stationY[i] = Units.inchesToMeters(152.19);
              break;
            case 7:
              stationY[i] = Units.inchesToMeters(174.19);
              break;
            case 8:
              stationY[i] = Units.inchesToMeters(196.19);
              break;
            default:
              stationY[i] = Units.inchesToMeters(20.19);
              break;
          }
        }

        List<Translation2d> stationToShelfWaypoints = new ArrayList<Translation2d>();
        List<Translation2d> shelfToStationWaypoints = new ArrayList<Translation2d>();

        if(SmartDashboard.getBoolean("Blue Alliance", true)) {
          stationToShelfWaypoints.add(new Translation2d(Units.inchesToMeters(84.595), Units.inchesToMeters(174.19)));
          stationToShelfWaypoints.add(new Translation2d(Units.inchesToMeters(151), Units.inchesToMeters(180.19)));
          stationToShelfWaypoints.add(new Translation2d((fieldLength / 2.0), Units.inchesToMeters(216.03)));
          stationToShelfWaypoints.add(new Translation2d((fieldLength / 2.0) + Units.inchesToMeters(61.36), Units.inchesToMeters(280.67) + (robotWidth / 2.0)));
          stationToShelfWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(142.25), Units.inchesToMeters(289.85)));

          shelfToStationWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(142.25), Units.inchesToMeters(289.85)));
          shelfToStationWaypoints.add(new Translation2d((fieldLength / 2.0) + Units.inchesToMeters(61.36), Units.inchesToMeters(280.67) + (robotWidth / 2.0)));
          shelfToStationWaypoints.add(new Translation2d((fieldLength / 2.0), Units.inchesToMeters(216.03)));
          shelfToStationWaypoints.add(new Translation2d(Units.inchesToMeters(151), Units.inchesToMeters(180.19)));
          shelfToStationWaypoints.add(new Translation2d(Units.inchesToMeters(84.595), Units.inchesToMeters(174.19)));
        } else {
          stationToShelfWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(84.595), Units.inchesToMeters(174.19)));
          stationToShelfWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(151), Units.inchesToMeters(180.19)));
          stationToShelfWaypoints.add(new Translation2d((fieldLength / 2.0), Units.inchesToMeters(216.03)));
          stationToShelfWaypoints.add(new Translation2d((fieldLength / 2.0) - Units.inchesToMeters(61.36), Units.inchesToMeters(280.67) + (robotWidth / 2.0)));
          stationToShelfWaypoints.add(new Translation2d(Units.inchesToMeters(142.25), Units.inchesToMeters(289.85)));

          shelfToStationWaypoints.add(new Translation2d(Units.inchesToMeters(142.25), Units.inchesToMeters(289.85)));
          shelfToStationWaypoints.add(new Translation2d((fieldLength / 2.0) - Units.inchesToMeters(61.36), Units.inchesToMeters(280.67) + (robotWidth / 2.0)));
          shelfToStationWaypoints.add(new Translation2d((fieldLength / 2.0), Units.inchesToMeters(216.03)));
          shelfToStationWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(151), Units.inchesToMeters(180.19)));
          shelfToStationWaypoints.add(new Translation2d(fieldLength - Units.inchesToMeters(84.595), Units.inchesToMeters(174.19)));
        }

        TrajectoryConfig limits = new TrajectoryConfig(Integer.MAX_VALUE, Integer.MAX_VALUE);
        Trajectory[] toStation = new Trajectory[9];
        Trajectory[] fromStation = new Trajectory[9];

        for(int i = 0; i < toStation.length; i++) {
          ControlVector shelf = new QuinticHermiteSpline.ControlVector(new double[]{shelfY, 0.0, 0.0}, new double[]{shelfX, 0.0, 0.0});
          ControlVector station = new QuinticHermiteSpline.ControlVector(new double[]{stationY[i], 0.0, 0.0}, new double[]{stationX, 0.0, 0.0});
          toStation[i] = TrajectoryGenerator.generateTrajectory(shelf, shelfToStationWaypoints, station, limits);
          fromStation[i] = TrajectoryGenerator.generateTrajectory(station, stationToShelfWaypoints, shelf, limits);
        }

        SmartDashboard.putNumber("Acceleration X", accelX);
        SmartDashboard.putNumber("Acceleration Y", accelX);
        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);
        SmartDashboard.putNumber("Position X", positionX);
        SmartDashboard.putNumber("Position Y", positionY);
        SmartDashboard.putNumber("Angular Acceleration", angleAccel);
        SmartDashboard.putNumber("Angular Velocity", angleVelocity);
        SmartDashboard.putNumber("Angle", angle);
        SmartDashboard.putNumber("Target Actuator", 0.0);
        SmartDashboard.putNumber("Target Potentiometer", 0.0);
        SmartDashboard.putBoolean("At Target", false);
        SmartDashboard.putNumber("Target Left Speed", targetLeftSpeed);
        SmartDashboard.putNumber("Target Right Speed", targetRightSpeed);

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        captures.add(new VideoCapture("http://roborio-3692-frc.local:1182/?action=stream"));
        captures.add(new VideoCapture("http://roborio-3692-frc.local:1181/?action=stream"));
        widths.add(SmartDashboard.getNumber("Top Width", 0.0));
        widths.add(SmartDashboard.getNumber("Bottom Width", 0.0));
        heights.add(SmartDashboard.getNumber("Top Height", 0.0));
        heights.add(SmartDashboard.getNumber("Bottom Height", 0.0));
        perPixelAngleX.add(SmartDashboard.getNumber("Top Pixel Angle X", 0.0));
        perPixelAngleX.add(SmartDashboard.getNumber("Bottom Pixel Angle X", 0.0));
        cameraAngles.add(SmartDashboard.getNumber("Top Angle", 0.0));
        cameraAngles.add(SmartDashboard.getNumber("Bottom Angle", 0.0));

        positionThread = new Thread(() -> {
      while(true) {
        leftSpeed = SmartDashboard.getNumber("Left Speed", 0.0);
        rightSpeed = SmartDashboard.getNumber("Right Speed", 0.0);

        prevAngularVelocity = angleVelocity;
        angleVelocity = leftSpeed - rightSpeed;
        angleVelocity /= robotRadius;

        //Integrate
        angle += angleVelocity * (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);
        //So it won't spend extra time going from something like 4 pi to -4 pi
        angle %= (Math.PI * 2);
        //Definition of derivative
        angleAccel = (angleVelocity - prevAngularVelocity) / (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);

        double totVelocity = Math.sqrt(Math.pow(leftSpeed, 2) + Math.pow(rightSpeed, 2));
        prevVelocityX = velocityX;
        prevVelocityY = velocityY;
        //Determine velocities from angle
        velocityX = Math.cos(angle) * totVelocity;
        velocityY = Math.sin(angle) * totVelocity;
        //Integrate
        positionX += velocityX * (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);
        positionY += velocityY * (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);
        //Derive
        accelX = (velocityX - prevVelocityX) / (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);
        accelY = (velocityY - prevVelocityY) / (SmartDashboard.getNumber("FPGA Time", 0.0) - currTime);

        //Update values (for people to see, roborio doesn't care)
        SmartDashboard.putNumber("Acceleration X", accelX);
        SmartDashboard.putNumber("Acceleration Y", accelX);
        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);
        SmartDashboard.putNumber("Position X", positionX);
        SmartDashboard.putNumber("Position Y", positionY);
        SmartDashboard.putNumber("Angular Acceleration", angleAccel);
        SmartDashboard.putNumber("Angular Velocity", angleVelocity);
        SmartDashboard.putNumber("Angle", angle);

        currTime = SmartDashboard.getNumber("FPGA Time", 0.0);

        //So the computer doesn't explode
        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {
          System.out.println("Interrupted");
        }
      }
    });
    //Won't persist through JVM exit
    positionThread.setDaemon(true);
    //It should be fine with normal priority
    positionThread.start();

    //If the computer can handle it, split into two threads to handle each capture seperately
    visionThread = new Thread(() -> {
        while(true) {
          for(int i = 0; i < captures.size(); i++) {
          //Empties current frames and grabs the next (that way image isn't old and thus useless)
            while(captures.get(i).grab()) {}
  
            //Will only happen if it was unable to grab a frame
            while(!captures.get(i).retrieve(img)) {
              try {
                Thread.sleep(250);
              } catch(InterruptedException e) {
                System.out.println("Interrupted");
              }

              captures.get(i).grab();
            }
  
            myTags = myDetector.detect(img);
  
            for(AprilTagDetection aDetection: myTags) {
              //Determines tag it's looking at
              AprilTag currTag = theTags.get(aDetection.getId() - 1);
  
              //Determines the other x coord that makes the triangle between the tag center and the image center an isoceles triangle
              double otherX = widths.get(i) - (aDetection.getCenterX() - (widths.get(i) / 2.0));
  
              //Since it's isoceles we can determine the angle between the camera and the points
              double largeCamAngle = perPixelAngleX.get(i) * Math.abs((aDetection.getCenterX() - otherX));
  
              //Angle between the center of the tag to the center of the image and the camera
              double centerAngle = (Math.PI - Math.abs(largeCamAngle)) / 2.0;
  
              //Distance between two corners of a recognized tag divided by image coordinates
              double perPixelLengthX = 0.1524 / (aDetection.getCornerX(0) - aDetection.getCornerX(2));
  
              //If you formed a right triangle between some unknown point, the camera, and the tag center this would be the camera angle
              double legitCamAngle = (Math.PI / 2.0) - centerAngle;
  
              //Trigonometry using known angles and pixel length to get each component
              double yComponent = (legitCamAngle / perPixelAngleX.get(i)) * perPixelLengthX;
              double xComponent = Math.tan(centerAngle) * yComponent;
              //Just in case
              double distance = Math.sqrt(Math.pow(yComponent, 2) + Math.pow(xComponent, 2));
  
              //yComponent is to the left of tag centre
              if(aDetection.getCenterX() > widths.get(i) / 2.0) {
                positionY = currTag.pose.getY() - yComponent;
                angle = ((Math.asin(xComponent / distance) - legitCamAngle) * -1) - cameraAngles.get(i);
              } else {
                positionY = currTag.pose.getY() + yComponent;
                angle = (Math.asin(xComponent / distance) - legitCamAngle) - cameraAngles.get(i);
              }
  
              //Might be flip-flopped
              if(aDetection.getId() <= 4) {
                positionX = currTag.pose.getX() - xComponent;
              } else {
                positionX = currTag.pose.getX() + xComponent;
              }
  
              SmartDashboard.putNumber("Position X", positionX);
              SmartDashboard.putNumber("Position Y", positionY);
              SmartDashboard.putNumber("Angle", angle);
            }
          }
  
          try {
            Thread.sleep(50);
          } catch(InterruptedException e) {
            System.out.println("Interrupted");
          }
        }
  
      });
      //Kinda needs a lot of resources
    visionThread.setPriority(Thread.MAX_PRIORITY);
    visionThread.setDaemon(true);
    visionThread.start();

    decider = new Thread(() -> {
      //True if it has a piece (goes towards scoring stations), false if not (goes towards shelf)
      boolean hasPiece = SmartDashboard.getBoolean("Has Piece", false);
      //Valid numbers are station 1-9
      int targetStation = (int) SmartDashboard.getNumber("Target Station", 1);
      //True if the decider should determing what drive to do
      boolean autoDrive = SmartDashboard.getBoolean("Auto Drive", false);
      //Must be reasonably close to teeter
      boolean teeter = SmartDashboard.getBoolean("Teeter", false);

      RamseteController controller = new RamseteController();
      DifferentialDriveKinematics helper = new DifferentialDriveKinematics(0.0);
      DifferentialDriveWheelSpeeds wheelSpeeds;

      if(autoDrive && !teeter) {
        Trajectory targTrajectory;

        if(hasPiece) {
          //Execute station code
          if(Math.abs(positionX - shelfX) < robotLength / 4.0 + shelfX && Math.abs(positionY - shelfY) < robotWidth / 4.0 + shelfY) {
            //Use pre-built
            targTrajectory = toStation[targetStation - 1];
          } else {
            //Create new trajectory
            ControlVector initial = new QuinticHermiteSpline.ControlVector(new double[]{positionY, velocityY, accelY}, new double[]{positionX, velocityX, accelX});
            ControlVector station = new QuinticHermiteSpline.ControlVector(new double[]{stationY[targetStation - 1], 0.0, 0.0}, new double[]{stationX, 0.0, 0.0});

            List<Translation2d> wayPoints = new ArrayList<Translation2d>();

            for(int i = 0; i < shelfToStationWaypoints.size(); i++) {
              if(shelfToStationWaypoints.get(i).getY() <= positionY) {
                wayPoints.add(shelfToStationWaypoints.get(i));
              }
            }

            targTrajectory = TrajectoryGenerator.generateTrajectory(initial, wayPoints, station, limits);
          }
        } else {
          //Execute shelf code
          if(Math.abs(positionX - stationX) < robotLength / 4.0 + stationX && Math.abs(positionY - stationYs[targetStation - 1]) < robotWidth / 4.0 + stationYs[targetStation - 1]) {
            //Execute pre-built
            targTrajectory = fromStation[targetStation - 1];

          } else {
            //Create new
            ControlVector initial = new QuinticHermiteSpline.ControlVector(new double[]{positionY, velocityY, accelY}, new double[]{positionX, velocityX, accelX});
            ControlVector shelf = new QuinticHermiteSpline.ControlVector(new double[]{shelfY, 0.0, 0.0}, new double[]{shelfX, 0.0, 0.0});

            List<Translation2d> wayPoints = new ArrayList<Translation2d>();

            for(int i = 0; i < stationToShelfWaypoints.size(); i++) {
              if(stationToShelfWaypoints.get(i).getY() >= positionY) {
                wayPoints.add(stationToShelfWaypoints.get(i));
              }
            }

            targTrajectory = TrajectoryGenerator.generateTrajectory(initial, wayPoints, shelf, limits);
          }
        }

        double tempTime = SmartDashboard.getNumber("FPGA Time", 0.0);
        for(double time = SmartDashboard.getNumber("FPGA Time", 0.0); time < targTrajectory.getTotalTimeSeconds() && SmartDashboard.getBoolean("Auto Drive", false); time = SmartDashboard.getNumber("FPGA Time", 0.0)) {
          Trajectory.State goalState = toStation[targetStation - 1].sample(time - tempTime);
          ChassisSpeeds adjustedSpeeds = controller.calculate(new Pose2d(positionY, positionX, new Rotation2d(angle)), goalState);
          wheelSpeeds = helper.toWheelSpeeds(adjustedSpeeds);

          SmartDashboard.putNumber("Target Left Speed", wheelSpeeds.leftMetersPerSecond);
          SmartDashboard.putNumber("Target Right Speed", wheelSpeeds.rightMetersPerSecond);

          try {
            Thread.sleep(5);
          } catch(InterruptedException e) {
            System.out.println("Interrupted");
          }
        }
      } else if(autoDrive && teeter) {
        //0 is upper left, goes counterclockwise
        boolean left = SmartDashboard.getBoolean("Left Teeter", false);
        boolean middle = SmartDashboard.getBoolean("Middle Teeter", false);
        boolean right = SmartDashboard.getBoolean("Right Teeter", false);
        double targetX = 0.0;
        double targetY = 0.0;
        double targetAngle = 0.0;
        double belowX = Units.inchesToMeters(114.94) - (robotLength / 2.0);
        double aboveX = ((fieldLength / 2.0) - Units.inchesToMeters(132.49)) + (robotLength / 2.0);

        if(SmartDashboard.getBoolean("Blue Alliance", true)) {
          if(Math.abs(positionX - belowX) < Math.abs(positionX - aboveX)) {
            targetX = belowX;
            targetAngle = 0.0;
          } else {
            targetX = aboveX;
            targetAngle = Math.PI;
          }
        } else {
          belowX = (fieldLength - belowX) + robotLength;
          aboveX = (fieldLength - aboveX) - robotLength;

          if(Math.abs(positionX - belowX) < Math.abs(positionX - aboveX)) {
            targetX = belowX;
            targetAngle = Math.PI;
          } else {
            targetX = aboveX;
            targetAngle = 0.0;
          }
        }

        if(right) {
          targetY = Units.inchesToMeters(59.39) + (robotWidth / 2.0);
        } else if(middle) {
          targetY = Units.inchesToMeters(108.015);
        } else if(right) {
          targetY = Units.inchesToMeters(156.64) - (robotWidth / 2.0);
        }

        Pose2d currentPose2d = new Pose2d(positionY, positionX, new Rotation2d(angle));
        Pose2d targetPose2d;
        //Pose2d targetPose2d = new Pose2d(targetY, targetX, new Rotation2d(targetAngle));

        if(SmartDashboard.getBoolean("Blue Alliance", true)) {
          if(positionX >= belowX && positionX <= aboveX && positionY <= Units.inchesToMeters(59.39) + (robotWidth / 2.0) && positionY >= Units.inchesToMeters(156.64) - (robotWidth / 2.0)) {
            targetPose2d = new Pose2d(positionY, Units.inchesToMeters(151), new Rotation2d(angle));
          } else {
            targetPose2d = new Pose2d(targetY, targetX, new Rotation2d(targetAngle));
          }
        } else {
          if(positionX <= belowX && positionX >= aboveX && positionY <= Units.inchesToMeters(59.39) + (robotWidth / 2.0) && positionY >= Units.inchesToMeters(156.64) - (robotWidth / 2.0)) {
            targetPose2d = new Pose2d(positionY, fieldLength - Units.inchesToMeters(151), new Rotation2d(angle));
          } else {
            targetPose2d = new Pose2d(targetY, targetX, new Rotation2d(targetAngle));
          }
        }

        ChassisSpeeds adjustedSpeeds = controller.calculate(currentPose2d, targetPose2d, 0.0, 0.0);
        wheelSpeeds = helper.toWheelSpeeds(adjustedSpeeds);

        SmartDashboard.putNumber("Target Left Speed", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Target Right Speed", wheelSpeeds.rightMetersPerSecond);
      }

      
    });
    decider.setPriority(Thread.MAX_PRIORITY);
    decider.setDaemon(true);
    decider.start();

    while(SmartDashboard.getNumber("FPGA Time", 0.0) - SmartDashboard.getNumber("Auto Start", -200.0) < 150) {
      try {
        Thread.sleep(1000);
      } catch(InterruptedException e) {
        System.out.println("Error in waiting in laptop main");
      }
    }
    }
}