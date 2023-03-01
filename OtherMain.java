import org.opencv.videoio;
import java.util.*;
import java.io.*;
import edu.wpi.first.apriltags.*;
import edu.wpi.first.networktables.*;

public class Main {
    ArrayList<VideoCapture> captures = new ArrayList<VideoCapture>();
    ArrayList<Double> widths = new ArrayList<Double>();
    ArrayList<Double> heights = new ArrayList<Double>();
    ArrayList<Double> perPixelAngleX = new ArrayList<Double>();
    ArrayList<Double> cameraAngles = new ArrayList<Double>();
    Mat img;

    //NEED radius to middle wheel in metres!!!
    final double robotRadius = 0.5;

    Thread positionThread;
    //Accel isn't actually important, just for the dashboard
    double accelX = 0.0;
    double accelY = 0.0;
    double velocityX = 0.0;
    double velocityY = 0.0;
    double positionX = 0.0;
    double positionY = 0.0;

    double angleAccel = 0.0;
    double angleVelocity = 0.0;
    double angle = Constants.blueAlliance ? 180.0 : 0.0;

    double currTime = Timer.getFPGATimestamp();
    double prevVelocityX = 0.0;
    double prevVelocityY = 0.0;
    double prevAngularVelocity = 0.0;

    Thread visionThread;
    Mat img;

    AprilTagDetector myDetector = new AprilTagDetector;
    ApriltagDetection[] myTags;

    AprilTagFieldLayout theLayout;
    AprilTag theTags;

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    public static void main(String[] args) {
        myDetector.addFamily("16h5");

        try {
            theLayout = new AprilTagFieldLayout("fieldLayout.json");
            theTags = theLayout.getTags();
        } catch(IOException e) {

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

        captures.add(new VideoCapture("http://roborio-3692-frc.local:1181/?action=stream"));
        captures.add(new VideoCapture("http://roborio-3692-frc.local:1182/?action=stream"));
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

        angle += angleVelocity * (Timer.getFPGATimestamp() - currTime);
        angle %= 360;
        angleAccel = (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - currTime);

        double totVelocity = Math.sqrt(Math.pow(leftSpeed, 2) + Math.pow(rightSpeed, 2));
        prevVelocityX = velocityX;
        prevVelocityY = velocityY;
        velocityX = Math.cos(angle) * totVelocity;
        velocityY = Math.sin(angle) * totVelocity;
        positionX += velocityX * (Timer.getFPGATimestamp() - currTime);
        positionY += velocityY * (Timer.getFPGATimestamp() - currTime);
        accelX = (velocityX - prevVelocityX) / (Timer.getFPGATimestamp() - currTime);
        accelY = (velocityY - prevVelocityY) / (Timer.getFPGATimestamp() - currTime);

        SmartDashboard.putNumber("Acceleration X", accelX);
        SmartDashboard.putNumber("Acceleration Y", accelX);
        SmartDashboard.putNumber("Velocity X", velocityX);
        SmartDashboard.putNumber("Velocity Y", velocityY);
        SmartDashboard.putNumber("Position X", positionX);
        SmartDashboard.putNumber("Position Y", positionY);
        SmartDashboard.putNumber("Angular Acceleration", angleAccel);
        SmartDashboard.putNumber("Angular Velocity", angleVelocity);
        SmartDashboard.putNumber("Angle", angle);

        currTime = Timer.getFPGATimestamp();

        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {

        }
      }
    });
    positionThread.isDaemon(true);
    positionThread.start();

    visionThread = new Thread(() -> {
      while(true) {
        for(int i = 0; i < captures.size(); i++) {
        //Empties current frames and grabs the next (synchronizes)
          while(captures.get(i).grab()) {}

          while(!captures.get(i).read(img)) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e);
          }

          myTags = detector.detect(img);

          for(AprilTagDetection aDetection: myTags) {
            //I'm keeping some of these calculations in case I need them
            //X runs horizontally acorss an image (ex: bottom left corner to bottom right corner only the x would change if perfectly lined up)
            AprilTag currTag = theTags.get(aDetection.getId() - 1);

            //double yLength = 0.0762;
            double xLength = 0.0762;
            
            double xAngleToCenter = (Math.abs(currTag.getCenterX() - (widths.get(i) / 2.0))) * perPixelAngleX.get(i);
            double xAngleToMid = (Math.abs(currTag.getCenterX() - (widths.get(i) / 2.0))) * perPixelAngleX.get(i);

            double cameraXAngle = 180 - (xAngleToCenter + xAngleToMid);

            //Law of Sines: sin(angle a) / length a == sin(angle b) / length b
            double xDistanceToCenter = Math.sin(xAngleToCenter) / (Math.sin(cameraXAngle) / xLength);

            double yComponent = Math.cos(xAngleToCenter) * xDistanceToCenter;
            double xComponent = Math.sin(xAngleToCenter) * xDistanceToCenter;

            double cameraCenterAngle = 90 - xAngleToCenter;

            angle = 90 - xAngle + cameraAngles.get(i);
            angle %= 360;

            if(currTag.getCenterX() < widths.get(i) / 2.0) {
              positionY = currTag.pose.getY() - yComponent;
            } else {
              positionY = currTag.pose.getY() + yComponent;
            }

            //Might be flipped
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

        }
      }

    });
    visionThread.setPriority(Thread.MAX_PRIORITY);
    visionThread.setDaemon(true);
    visionThread.start();
    }
}
