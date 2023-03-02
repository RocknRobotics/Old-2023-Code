import org.opencv.videoio;
import java.util.*;
import java.io.*;
import edu.wpi.first.apriltags.*;
import edu.wpi.first.networktables.*;

/*
TODO:
Get the robot radius
Get the numbers for the actuator/potentiometer
Spend time crunching numbers for positions
*/

public class Main {
    ArrayList<VideoCapture> captures = new ArrayList<VideoCapture>();
    ArrayList<Double> widths = new ArrayList<Double>();
    ArrayList<Double> heights = new ArrayList<Double>();
    ArrayList<Double> perPixelAngleX = new ArrayList<Double>();
    ArrayList<Double> cameraAngles = new ArrayList<Double>();
    Mat img;

    //NEED radius to middle wheel in metres!!!
    final double robotRadius = 0.5;
    //Change if needed
    final boolean blueAlliance = true;
    //Where the robot needs to be to be clear of the barrier (metres)
    final double clearBarrierPosition = 0.0;
    //Field length (x)
    final double fieldLength = 16.54175;
    //Field width(y)
    final double fieldWidth = 8.0137;
    //The imaginary line in the field where the robot should go to so that it just barely lines up with the corner station
    final double cornerLineY = 0.0;
    //Line for lining up x in front of station
    final double stationLine = 0.0;
    //Station y position tolerance
    final double stationYTolerance = 0.0;
    //Station y positions (different from april tags since those are only like every other station)
    final double[] stationYs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //Y position of the shelf
    final double shelfY = 0.0;
    //X position of the shelf
    final double shelfX = 0.0;

    Thread positionThread;
    //Accel isn't actually important in this class, just for the dashboard
    double accelX = 0.0;
    double accelY = 0.0;
    double velocityX = 0.0;
    double velocityY = 0.0;
    double positionX = 0.0;
    double positionY = 0.0;

    double angleAccel = 0.0;
    double angleVelocity = 0.0;
    double angle = blueAlliance ? 180.0 : 0.0;

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

    Thread decider;

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
        SmartDashboard.putNumber("Target Actuator", 0.0);
        SmartDashboard.putNumber("Target Potentiometer", 0.0);
        SmartDashboard.putBoolean("At Target", false);

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
        //Empties current frames and grabs the next (that way image isn't old and thus useless)
          while(captures.get(i).grab()) {}

          while(!captures.get(i).read(img)) {
            try {
                Thread.sleep(10);
            } catch(InterruptedException e);
          }

          myTags = detector.detect(img);

          for(AprilTagDetection aDetection: myTags) {
            AprilTag currTag = theTags.get(aDetection.getId() - 1);

            double otherX = widths.get(i) - (aDetection.getCenterX() - (widths.get(i) / 2.0));

            double largeCamAngle = perPixelAngleX.get(i) * Math.abs((aDetection.getCenterX() - otherX));

            double centerAngle = (180 - Math.abs(largCamAngle)) / 2.0;

            //Distance between two corners of a recognized tag divided by image coordinates
            double perPixelLengthX = 0.1524 / (aDetection.getCornerX(0) - aDetection.getCornerX(2));

            double legitCamAngle = 90 - centerAngle;

            double yComponent = (legitCamAngle / perPixelAngleX) * perPixelLengthX;
            double xComponent = Math.tan(centerAngle) * yComponent;
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

        }
      }

    });
    visionThread.setPriority(Thread.MAX_PRIORITY);
    visionThread.setDaemon(true);
    visionThread.start();

    decider = new Thread(() -> {
      //1, 2, and 3 are reserved for bottom node, middle node, and top node respectively
      //0 is reserved for moving it just in front of robot (for picking up ground pieces)
      //4 is reserved for moving it to pick things up from game piece shelf
      //Just realized, zero straight up won't be used???
      int level = SmartDashboard.getNumber("Target Arm State", 0);
      boolean hasPiece = SmartDashboard.getBoolean("Has Piece", false);
      int targetStation = SmartDashboard.getNumber("Target Station", 1);
      boolean autoDrive = SmartDashboard.getBoolean("Auto Drive", false);

      boolean barrierStatement = (blueAlliance ? (positionX >= clearBarrierPosition) : (positionX <= fieldLength - clearBarrierPosition));
      boolean cornerStatement = (positionY >= cornerLineY);
      boolean lineStatement = (blueAlliance ? (Math.abs(positionX - stationLine) > stationLine) : (Math.abs(positionX - (fieldLength - stationLine)) > stationLineTolerance));
      boolean stationStatement = Math.abs(positionY - stationYs[targetStation - 1]) > stationYTolerance;
      boolean shelfStatement = (positionY <= shelfY);

      if(autoDrive) {
        if(hasPiece) {
          switch(level) {
            case 1:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
            case 2:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
            case 3:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
            default:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
          }
          
          if(blueAlliance) {
            if(barrierStatement && cornerStatement) {
              SmartDashboard.putNumber("Target Angle", 0.0);
              SmartDashboard.putNumber("Target Position X", clearBarrierPosition - 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(cornerStatement && !lineStatement) {
              SmartDashboard.putNumber("Target Angle", 90.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", cornerLineY - 0.5);
            } else if(lineStatement) {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", stationLine);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(stationStatement) {
              SmartDashboard.putNumber("Target Angle", 90.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", stationYs[targetStation - 1]);
            } else {
              SmartDashboard.putNumber("Target Angle", 0.0);
              SmartDashboard.putNumber("Target Position X", positionX - 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
              SmartDashboard.putBoolean("At Target", true);

              while(SmartDashboard.getBoolean("Has Piece", false)) {
                try {
                  Thread.sleep(50);
                } catch(InterruptedException e) {

                }
              }

              SmartDashboard.putBoolean("At Target", false);
            }
          } else {
            if(barrierStatement && cornerStatement) {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", (fieldLength - clearBarrierPosition) + 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(cornerStatement && !lineStatement) {
              SmartDashboard.putNumber("Target Angle", 90.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", cornerLineY - 0.5);
            } else if(lineStatement) {
              SmartDashboard.putNumber("Target Angle", 0.0);
              SmartDashboard.putNumber("Target Position X", fieldLength - stationLine);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(stationStatement) {
              SmartDashboard.putNumber("Target Angle", 90.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", stationYs[targetStation - 1]);
            } else {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", positionX + 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
              SmartDashboard.putBoolean("At Target", true);

              while(SmartDashboard.getBoolean("Has Piece", false)) {
                try {
                  Thread.sleep(50);
                } catch(InterruptedException e) {

                }
              }

              SmartDashboard.putBoolean("At Target", false);
            }
          }
        } else {
          switch(level) {
            case 0:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
            case 4:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
            default:
              SmartDashboard.putNumber("Target Actuator", 0.0);
              SmartDashboard.putNumber("Target Potentiometer", 0.0);
              break;
          }

          if(blueAlliance) {
            if(barrierStatement && shelfStatement) {
              SmartDashboard.putNumber("Target Angle", 0.0);
              SmartDashboard.putNumber("Target Position X", (fieldLength - clearBarrierPosition) + 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(shelfStatement) {
              SmartDashboard.putNumber("Target Angle", 270.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", shelfY);
            } else {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", shelfX);
              SmartDashboard.putNumber("Target Position Y", positionY);
              SmartDashboard.putBoolean("At Target", true);

              while(!SmartDashboard.putBoolean("Has Piece", false)) {
                try {
                  Thread.sleep(50);
                } catch(InterruptedException e) {

                }
              }

              SmartDashboard.putBoolean("At Target", false);
            }
          } else {
            if(barrierStatement && shelfStatement) {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", clearBarrierPosition - 0.5);
              SmartDashboard.putNumber("Target Position Y", positionY);
            } else if(shelfStatement) {
              SmartDashboard.putNumber("Target Angle", 270.0);
              SmartDashboard.putNumber("Target Position X", positionX);
              SmartDashboard.putNumber("Target Position Y", shelfY);
            } else {
              SmartDashboard.putNumber("Target Angle", 180.0);
              SmartDashboard.putNumber("Target Position X", fieldLength - shelfX);
              SmartDashboard.putNumber("Target Position Y", positionY);
              SmartDashboard.putBoolean("At Target", true);

              while(!SmartDashboard.putBoolean("Has Piece", false)) {
                try {
                  Thread.sleep(50);
                } catch(InterruptedException e) {

                }
              }

              SmartDashboard.putBoolean("At Target", false);
            }
          }
        }
      }
    });
    decider.setPriority(Thread.MAX_PRIORITY);
    decider.setDaemon(true);
    decider.start();
    }
}
