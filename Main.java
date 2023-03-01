// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.*;
import java.io.*;
import org.opencv.core.Mat;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.apriltag.*;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  //I might add another for camera position relative to the center of the robot if needed, for now this should be good enough
  public static ArrayList<CvSink> Sinks = new ArrayList<CvSink>();
  public static ArrayList<Double> widths = new ArrayList<Double>();
  public static ArrayList<Double> heights = new ArrayList<Double>();
  public static ArrayList<Double> perPixelAngleX = new ArrayList<Double>();
  //public static ArrayList<Double> perPixelAngleY = new ArrayList<Double>();
  public static ArrayList<Double> cameraAngles = new ArrayList<Double>();

  public static double experimentalVelocityConstant = 0.0;
  //Radius to center wheel (metres)
  //NEED ACTUAL VALUE
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

  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);

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

    positionThread = new Thread(() -> {
      while(true) {
        prevAngularVelocity = angleVelocity;
        angleVelocity = Robot.leftSpeed * experimentalVelocityConstant - Robot.rightSpeed * experimentalVelocityConstant;
        angleVelocity /= robotRadius;

        angle += angleVelocity * (Timer.getFPGATimestamp() - currTime);
        angle %= 360;
        angleAccel = (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - currTime);

        double totVelocity = Math.sqrt(Math.pow(Robot.leftSpeed * experimentalVelocityConstant, 2) + Math.pow(Robot.rightSpeed * experimentalVelocityConstant, 2));
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
        for(int i = 0; i < Sinks.size(); i++) {
          Sinks.get(i).grabFrame(img);

          myTags = detector.detect(img);

          for(AprilTagDetection aDetection: myTags) {
            //I'm keeping some of these calculations in case I need them
            //X runs horizontally acorss an image (ex: bottom left corner to bottom right corner only the x would change if perfectly lined up)
            AprilTag currTag = theTags.get(aDetection.getId() - 1);

            //double xPixelLength = 0.1524 / Math.abs(currTag.getCornerX(1) - currTag.getCornerX(3));
            //double yPixelLength = 0.1524 / Math.abs(currTag.getCornerY(1) - currTag.getCornerY(3));

            //double xAngle = (Math.abs(currTag.getCornerX(1) - currTag.getCornerX(3)) / 2.0) * perPixelAngleX.get(i);
            //double yAngle = (Math.abs(currTag.getCornerY(1) - currTag.getCornerY(3)) / 2.0) * perPixelAngleY.get(i);

            //double yLength = 0.0762;
            double xLength = 0.0762;
            
            double xAngleToCenter = (Math.abs(currTag.getCenterX() - (widths.get(i) / 2.0))) * perPixelAngleX.get(i);
            //double yAngleToCenter = (Math.abs(((currTag.getCornerX(1) - currTag.getCornerX(3)) / 2.0) - (widths.get(i) / 2.0))) * perPixelAngleY.get(i);
            double xAngleToMid = (Math.abs(currTag.getCenterX() - (widths.get(i) / 2.0))) * perPixelAngleX.get(i);
            //double yAngleToMid = (Math.abs(((currTag.getCornerY(1) - currTag.getCornerY(3)) / 2.0) - (widths.get(i) / 2.0))) * perPixelAngleY.get(i);

            double cameraXAngle = 180 - (xAngleToCenter + xAngleToMid);
            //double cameraYAngle = 180 - (yAngleToCenter + yAngleToMid);

            //Law of Sines: sin(angle a) / length a == sin(angle b) / length b
            double xDistanceToCenter = Math.sin(xAngleToCenter) / (Math.sin(cameraXAngle) / xLength);
            //double yDistanceToCenter = Math.sin(yAngleToCenter) / (Math.sin(cameraYAngle) / yLength);

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
