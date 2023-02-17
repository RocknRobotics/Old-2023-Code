
/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import java.io.OutputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Robot extends TimedRobot {

  // Definitions for the hardware. Change this if you change what stuff you have
  // plugged in
  // drive
  CANSparkMax driveLeftA = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(2, MotorType.kBrushless);
  // accelerometer
  Accelerometer accelerometer = new BuiltInAccelerometer();

  PS4Controller ps1 = new PS4Controller(0);// drive controller

  double autoStart = 0;
  boolean goForAuto = false;
  boolean fast = false;
  // camera

  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure motors to turn correct direction. You may have to invert some of
    // your motors
    driveLeftA.setInverted(true);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    driveRightA.burnFlash();
    driveRightB.setInverted(false);
    driveRightB.burnFlash();
    fast = false;

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    // add a thing on the dashboard to turn off auto if needed
    // SmartDashboard.put
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);

    // accelerometers
    SmartDashboard.putNumber("accelerometer (left/right)", accelerometer.getX());

    SmartDashboard.putNumber("accelerometer (Fowards/Backwards)", accelerometer.getY());

  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if (goForAuto) {

      // series of timed events making up the flow of auto
      if (autoTimeElapsed < 4) {
        // spit out the ball for three seconds
        // intake.set(ControlMode.PercentOutput, -1);

      } else if (autoTimeElapsed < 7) {
        // stop spitting out the ball and drive backwards *slowly* for three seconds
        // intake.set(ControlMode.PercentOutput, 0);

        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      } else {
        // do nothing for the rest of auto
        // intake.set(ControlMode.PercentOutput, 0);

        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);

      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Set up arcade steer
    double forward = 0;
    double turn = 0;

    // xbox
    if(ps1.getTriangleButtonReleased()){
      fast = !fast;
    }
    if (!fast) {
      // regular mode & left trigger backward & right trigger foward
      forward = (-1 * (ps1.getR2Axis() / 4)) + (ps1.getL2Axis() / 4);
      if (Math.abs(ps1.getRightX()) > .15) {
        turn = ps1.getRightX() / 4;// left stick steer x-axis
      }
    } else {
      // fast mode & left trigger backward & right trigger foward
      forward = (-1 * (ps1.getR2Axis())) + (ps1.getL2Axis());
      if (Math.abs(ps1.getRightX()) > .15) {
        turn = ps1.getRightX();// left stick steer x-axis
      }
    }
    double driveLeftPower = (forward + turn);
    double driveRightPower = (forward - turn);

    driveLeftA.set(driveLeftPower / 2);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(driveRightPower / 2);
    driveRightB.follow(driveRightA);
  }

  @Override
  public void disabledInit() {
    // On disable turn off everything
    // done to solve issue with motors "remembering" previous setpoints after
    // reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    // intake.set(ControlMode.PercentOutput, 0);
  }

}

/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import java.io.OutputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Robot extends TimedRobot {

  // Definitions for the hardware. Change this if you change what stuff you have
  // plugged in
  // drive
  CANSparkMax driveLeftA = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(2, MotorType.kBrushless);
  // accelerometer
  Accelerometer accelerometer = new BuiltInAccelerometer();

  PS4Controller ps1 = new PS4Controller(0);// drive controller

  double autoStart = 0;
  boolean goForAuto = false;
  boolean fast = false;
  // camera

  Thread m_visionThread;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure motors to turn correct direction. You may have to invert some of
    // your motors
    driveLeftA.setInverted(true);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(true);
    driveLeftB.burnFlash();
    driveRightA.setInverted(false);
    driveRightA.burnFlash();
    driveRightB.setInverted(false);
    driveRightB.burnFlash();
    fast = false;

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    // add a thing on the dashboard to turn off auto if needed
    // SmartDashboard.put
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);

    // accelerometers
    SmartDashboard.putNumber("accelerometer (left/right)", accelerometer.getX());

    SmartDashboard.putNumber("accelerometer (Fowards/Backwards)", accelerometer.getY());

  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // get time since start of auto
    double autoTimeElapsed = Timer.getFPGATimestamp() - autoStart;
    if (goForAuto) {

      // series of timed events making up the flow of auto
      if (autoTimeElapsed < 4) {
        // spit out the ball for three seconds
        // intake.set(ControlMode.PercentOutput, -1);

      } else if (autoTimeElapsed < 7) {
        // stop spitting out the ball and drive backwards *slowly* for three seconds
        // intake.set(ControlMode.PercentOutput, 0);

        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      } else {
        // do nothing for the rest of auto
        // intake.set(ControlMode.PercentOutput, 0);

        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);

      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Set up arcade steer
    double forward = 0;
    double turn = 0;

    // xbox
    if(ps1.getTriangleButtonReleased()){
      fast = !fast;
    }
    if (!fast) {
      // regular mode & left trigger backward & right trigger foward
      forward = (-1 * (ps1.getR2Axis() / 4)) + (ps1.getL2Axis() / 4);
      if (Math.abs(ps1.getRightX()) > .15) {
        turn = ps1.getRightX() / 4;// left stick steer x-axis
      }
    } else {
      // fast mode & left trigger backward & right trigger foward
      forward = (-1 * (ps1.getR2Axis())) + (ps1.getL2Axis());
      if (Math.abs(ps1.getRightX()) > .15) {
        turn = ps1.getRightX();// left stick steer x-axis
      }
    }
    double driveLeftPower = (forward + turn);
    double driveRightPower = (forward - turn);

    driveLeftA.set(driveLeftPower / 2);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(driveRightPower / 2);
    driveRightB.follow(driveRightA);
  }

  @Override
  public void disabledInit() {
    // On disable turn off everything
    // done to solve issue with motors "remembering" previous setpoints after
    // reenable
    driveLeftA.set(0);
    driveLeftB.set(0);
    driveRightA.set(0);
    driveRightB.set(0);
    // intake.set(ControlMode.PercentOutput, 0);
  }

}
