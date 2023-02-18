
/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/

package frc.robot;

import java.io.OutputStream;

//Motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//Pneumatics
import edu.wpi.first.wpilibj.DoubleSolenoid;

//Images
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//Camera
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;

//Accelerometer
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

//Potentiometer
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//timer
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Controller
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;

public class Robot extends TimedRobot {

  // Definitions for the hardware. Change this if you change what stuff you have
  // plugged in
  // drive motors
  CANSparkMax driveLeftA = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(2, MotorType.kBrushless);

  //Linear actuator
  CANSparkMax armActuator = new CANSparkMax(7, MotorType.kBrushless);

  //Arm Extension 
  CANSparkMax armExtension = new CANSparkMax(13, MotorType.kBrushless);

  //Pneumatics
  DoubleSolenoid clawSolenoid1 = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid clawSolenoid2 = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 1, 0);
  

  // accelerometer
  Accelerometer accelerometer = new BuiltInAccelerometer();

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(0);

  //Controller
  PS4Controller ps1 = new PS4Controller(0);

  //Camera
  Thread m_visionThread;

  double autoStart = 0;
  boolean goForAuto = false;
  boolean fast = false;

 //Station (starting position)
  int station = -1;

  //Finals
  //Used for teeter totter balancing
  final double accelProportion = 1.0;
  //Timestamp that it takes the robot to drive back to a new piece
  final double autoBackUpTime = 0.0;
  //Timestamp for the robot to turn around so it can grab a piece
  final double turnTime = 0.0;


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

    //Station number---All start in front of a cone stand (for now)
    //Left far end
    SmartDashboard.putBoolean("Station 1", true);
    //Middle
    SmartDashboard.putBoolean("Station 2", true);
    //Right far end
    SmartDashboard.putBoolean("Station 3", true);

  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
    
    if(SmartDashboard.getBoolean("Station 1", true)) {
      scoreCone();

      //This and other instances of -1 drive power might have to be changed to 1
      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveRightA);

      while(Timer.getFPGATimestamp() - autoStart <= autoBackUpTime) {}

      driveLeftA.set(0);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1);
      driveLeftB.follow(driveRightA);

      while(Timer.getFPGATimestamp() - autoStart <= turnTime) {}
      
      /**
       * Grab piece code
       */

       

    } else if(SmartDashboard.getBoolean("Station 2", true)) {

    } else if(SmartDashboard.getBoolean("Station 3", true)) {

    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (goForAuto) {

      // series of timed events making up the flow of auto
      if (Timer.getFPGATimestamp() - autoStart < 4) {
        // spit out the ball for three seconds
        // intake.set(ControlMode.PercentOutput, -1);

      } else if (Timer.getFPGATimestamp() - autoStart < 7) {
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

    if(Timer.getFPGATimestamp() - autoStart >= 215) {
      //Maybe this needs to be getZ instead of getX
      driveLeftA.set(accelProportion * -1 * accelerometer.getX());
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveRightB);
    }
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

  public void scoreCone() {
    //Code for scoring a cone
  }
  public void scoreCube() {
    //Code for scoring a cube
  }
}
