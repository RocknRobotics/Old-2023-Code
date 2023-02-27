
/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/
package frc.robot;

import java.io.OutputStream;

import java.text.*;
import java.util.*;

//Motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;

//Pneumatics
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

//Images
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

//Camera
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Accelerometer
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;

//Potentiometer
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//timer
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Controller
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;

public class Robot extends TimedRobot {

  DecimalFormat fmt = new DecimalFormat("#.00");
  // Definitions for the hardware. Change this if you change what stuff you have
  // plugged in
  // drive motors
  CANSparkMax driveLeftA = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveLeftB = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax driveRightA = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(2, MotorType.kBrushless);

  //Linear actuator
  CANSparkMax armActuator = new CANSparkMax(7, MotorType.kBrushed);

  //Arm Extension 
  CANSparkMax armExtension = new CANSparkMax(8, MotorType.kBrushed);

  //Camera Servo
  Servo cameraServo = new Servo(0);

  //Pneumatics
  PneumaticsControlModule PneumaticsControl = new PneumaticsControlModule();
  Compressor PneumaticsCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid clawSolenoid1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid clawSolenoid2 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
  

  // accelerometer
  //Parameter order
  //SPI.Port---The port used to connect to the navX (can be a I2C.Port instead) (this might just be a number, I'm not sure)
  //int---the bitrate of the sensor (max 2,000,000)
  //int---the update rate of the sensor sending us data (4 - 200)
  /*AHRS accelerometer = new AHRS(Port.kMXP, (byte) 4);
  double accelOffsetX = 0.0;
  double accelOffsetY = 0.0;
  double accelOffsetZ = 0.0;
  double accelX = 0.0;
  double accelY = 0.0;
  double accelZ = 0.0;
  double velocityX = 0.0;
  double velocityY = 0.0;
  double velocityZ = 0.0;
  double positionX = 0.0;
  double positionY = 0.0;
  double positionZ = 0.0;

  double angularAccel = 0.0;
  double angularVelocity = 0.0;
  double angle = 0.0;
  double servoAngle = 0;

  double accelTime = Timer.getFPGATimestamp();

  Thread accelThread;*/

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(1);
  AnalogPotentiometer armExtensionPotentiometer = new AnalogPotentiometer(0);

  //Arm threads
  Thread armAngleThread;
  double targetArmAngle = 0.0;
  Thread armExtensionThread;
  double targetExtensionLength = 0.0;

  //Controller
  PS4Controller ps1 = new PS4Controller(0);
  PS4Controller ps2 = new PS4Controller(1);

  //Camera
  Thread m_visionThread;
  UsbCamera topCamera;
  UsbCamera bottomCamera;
  NetworkTableEntry cameraSelection;
  boolean topcam;


  double prev = 0;
  double autoStart = 0;
  boolean goForAuto = false;
  boolean fast = false;
  boolean closed = false;
  double leftSpeed = 0;
  double rightSpeed = 0;
  boolean stopped1 = false;
  boolean stopped2 = false;

 //Station (starting position) and cone/cube
  int station = -1;

  //Hopefully Raspberry Pi code
  //Raspberry Pi will house accelerometers and usb cameras
  //Raspberry Pi will only return acceleration, velocity, position, angular accel, angular velocity, and angle
  //(8 byte accel, 8 byte velocity, 8 byte position) * 3, 8 bytes anglular accel, 8 bytes angular velocity, 8 bytes angle
  //96 bytes total
  //Nevermind, I'll try and use network tables to update directly from the raspberry pi

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Configure motors to turn correct direction. You may have to invert some of
    // your motors
    driveLeftA.setInverted(false);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(false);
    driveLeftB.burnFlash();
    driveRightA.setInverted(true);
    driveRightA.burnFlash();
    driveRightB.setInverted(true);
    driveRightB.burnFlash();
    armActuator.setInverted(false);
    armActuator.burnFlash();
    armExtension.setInverted(false);
    armExtension.burnFlash();
    
    driveLeftA.setClosedLoopRampRate(200);
    driveLeftB.setClosedLoopRampRate(200);
    driveRightA.setClosedLoopRampRate(200);
    driveRightB.setClosedLoopRampRate(200);
    driveLeftA.burnFlash();
    driveLeftB.burnFlash();
    driveRightA.burnFlash();
    driveRightB.burnFlash();
    
    fast = false;
    closed = false;
    stopped1 = false;
    stopped2 = false;

    //Pneumatics
    PneumaticsControl.enableCompressorAnalog(100, 120);
    PneumaticsCompressor.enableAnalog(100, 120);

    //cameras
    bottomCamera = CameraServer.startAutomaticCapture("Bottom", 0);
    topCamera = CameraServer.startAutomaticCapture("Top", 1);
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    cameraSelection.setString(bottomCamera.getName());
    topcam = false;

    // add a thing on the dashboard to turn off auto if needed
    // SmartDashboard.put
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);

    // accelerometers
    SmartDashboard.putNumber("Acceleration X", 0.0);
    SmartDashboard.putNumber("Acceleration Z", 0.0);
    SmartDashboard.putNumber("Acceleration Y", 0.0);
    SmartDashboard.putNumber("Velocity X", 0.0);
    SmartDashboard.putNumber("Velocity Z", 0.0);
    SmartDashboard.putNumber("Velocity Y", 0.0);
    SmartDashboard.putNumber("Position X", 0.0);
    SmartDashboard.putNumber("Position Z", 0.0);
    SmartDashboard.putNumber("Position Y", 0.0);
    SmartDashboard.putNumber("Angular Acceleration", 0.0);
    SmartDashboard.putNumber("Angular Velocity", 0.0);
    SmartDashboard.putNumber("Angle", 0.0);
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
    SmartDashboard.putNumber("arm extension", armExtensionPotentiometer.get());

    //Station number---All start in front of a cone stand (for now)
    //Left far end
    SmartDashboard.putBoolean("Station 1", false);
    //Middle
    SmartDashboard.putBoolean("Station 2", false);
    //Right far end
    SmartDashboard.putBoolean("Station 3", false);
    //Cone or Cube to pick up
    SmartDashboard.putBoolean("Cone", false);
    SmartDashboard.putBoolean("Cube", false);

    /*accelThread = new Thread(() -> {
      while(1 != 0) {
        accelX = accelerometer.getWorldLinearAccelX() - accelOffsetX;
        accelY = accelerometer.getWorldLinearAccelY() - accelOffsetY;
        accelZ = accelerometer.getWorldLinearAccelZ() - accelOffsetZ;
        velocityX += (Timer.getFPGATimestamp() - accelTime) * accelX;
        velocityY += (Timer.getFPGATimestamp() - accelTime) * accelY;
        velocityZ += (Timer.getFPGATimestamp() - accelTime) * accelZ;
        positionX += (Timer.getFPGATimestamp() - accelTime) * velocityX;
        positionY += (Timer.getFPGATimestamp() - accelTime) * velocityY; 
        positionZ += (Timer.getFPGATimestamp() - accelTime) * velocityZ;

        double prevAngle = angle;
        double prevAngularVelocity = angularVelocity;
        angle = accelerometer.getYaw() < 0 ? accelerometer.getYaw() + 360 : accelerometer.getYaw();
        angularVelocity = (angle - prevAngle > 180 ? -1 * (prevAngle - angle) : angle - prevAngle) / (Timer.getFPGATimestamp() - accelTime);
        angularAccel = (angularVelocity - prevAngularVelocity > 180 ? -1 * (prevAngularVelocity - angularVelocity) : angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - accelTime);

        accelTime = Timer.getFPGATimestamp();

        try {
          Thread.sleep(250);
        } catch(InterruptedException e) {
          
        }
      }
    });
    accelThread.setPriority(Thread.MIN_PRIORITY);
    accelThread.setDaemon(true);
    accelThread.start();*/

    armAngleThread = new Thread(() -> {
      while(Math.abs(armPotentiometer.get() - targetArmAngle) > Constants.armAngleTolerance) {
        if(armPotentiometer.get() < targetArmAngle) {
          armActuator.set(-1);
        } else {
          armActuator.set(1);
        }
      }

      armActuator.set(0);

      try {
          Thread.sleep(100);
        } catch(InterruptedException e) {
          
        }
    });
    armAngleThread.setPriority(Thread.MIN_PRIORITY);
    armAngleThread.setDaemon(true);

    armExtensionThread = new Thread(() -> {
      while(Math.abs(armExtension.get() - targetExtensionLength) > Constants.armLengthTolerance) {
        if(armExtension.get() < targetExtensionLength) {
          armExtension.set(-1);
        } else {
          armExtension.set(1);
        }
      }

      armExtension.set(0);

      try {
          Thread.sleep(100);
        } catch(InterruptedException e) {
          
        }
    });
    armExtensionThread.setPriority(Thread.MIN_PRIORITY);
    armExtensionThread.setDaemon(true);
  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto

    targetArmAngle = 0.5;
    armAngleThread.start();
    targetExtensionLength = 0.755;
    armExtensionThread.start();

    clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    clawSolenoid2.set(DoubleSolenoid.Value.kReverse);

    targetArmAngle = 0.05;
    armAngleThread.start();
    targetExtensionLength = 0.15;
    armExtensionThread.start();

    double currTime = Timer.getFPGATimestamp();
    driveLeftA.set(-1);
    driveRightA.set(-1);

    while(Timer.getFPGATimestamp() - currTime < Constants.autoBackup) {
      try {
        Thread.sleep(10);
      } catch(InterruptedException e) {

      }
    }

    currTime = Timer.getFPGATimestamp();

    if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 1;

      if(Constants.blueAlliance) {
        driveLeftA.set(1);
        driveRightA.set(-1);
      } else {
        driveLeftA.set(-1);
        driveRightA.set(1);
      }
    } else if(SmartDashboard.getBoolean("Station 2", true)) {
      station = 2;
      
    }

    if(!SmartDashboard.getBoolean("Go For Auto", true)) {
      return;
    }

    if(SmartDashboard.getBoolean("Has Cone", true)) {
      //I'm not entirely sure but I'm hoping this will reset the variable value
      SmartDashboard.putBoolean("Has Cone", false);
      scoreConeTop();
    } else if(SmartDashboard.getBoolean("Has Cube", true)) {
      //I might change this to listen to the controller to know when it has a cube/cone and stuff, it depends
      SmartDashboard.putBoolean("Has Cube", false);
      scoreCubeTop();
    }

    //This and other instances of -1 drive power might have to be changed to 1
    //goTo(Constants.autoPieceX, Constants.autoPieceY[station - 1], 180.0);
      
    if(SmartDashboard.getBoolean("Grab Cone", true)) {
      SmartDashboard.putBoolean("Grab Cone", false);
      grabCone();
    } else if(SmartDashboard.getBoolean("Grab Cube", true)) {
      SmartDashboard.putBoolean("Grab Cube", false);
      grabCube();
    }

  //balanceOnTeeter(14.9, false, true, false);
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
    //accelerometer.resetDisplacement();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //accelThread.start();
    //disable and enable
    

    if (ps1.getPSButtonPressed()) {
      stopped1 = !stopped1;
    }

    if (ps2.getPSButtonPressed()) {
      stopped2 = !stopped2;
    }
    

    // Set up arcade steer
    double forward = 0;
    double turn = 0;

    //System.out.println(driveLeftA.getClosedLoopRampRate());
    //System.out.println(driveLeftA.getOpenLoopRampRate());

    // xbox
    if (!stopped1) {
      if(ps1.getTriangleButtonReleased()){
        fast = !fast;
      }
      if (!fast) {
        // regular mode & left trigger backward & right trigger foward
        forward = (-1 * (ps1.getL2Axis() / 4)) + (ps1.getR2Axis() / 4);
        if (Math.abs(ps1.getLeftX()) > .15) {
          turn = ps1.getLeftX() / 4;// right stick steer x-axis
        }
      } else {
        // fast mode & left trigger backward & right trigger foward
        forward = (-1 * (ps1.getL2Axis())) + (ps1.getR2Axis());
        if (Math.abs(ps1.getLeftX()) > .15) {
          turn = ps1.getLeftX();// right stick steer x-axis
        }
      }
      turn *= 0.9;
      double driveLeftPower = (forward + turn);
      double driveRightPower = (forward - turn);

      if (fast) {
        leftSpeed = leftSpeed + (driveLeftPower - leftSpeed) / 20;
        rightSpeed = rightSpeed + (driveRightPower - rightSpeed) / 20;
      } else {
        leftSpeed = leftSpeed + (driveLeftPower - leftSpeed) / 10;
        rightSpeed = rightSpeed + (driveRightPower - rightSpeed) / 10;
      }


      driveLeftA.set(leftSpeed / 2);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(rightSpeed / 2);
      driveRightB.follow(driveRightA);
    }

    //arm testing
    //highest arm angle is 0.442
    //lowest is 
    if (!stopped2) {
      if (ps2.getL2Axis() > 0.5) {
        armActuator.set(-1);
      } else if (ps2.getR2Axis() > 0.5) {
        armActuator.set(1);
      } else {
        armActuator.set(0);
      }

      if (ps2.getLeftY() < -0.5 && armExtensionPotentiometer.get() < 0.755) {
        armExtension.set(1);
      } else if (ps2.getLeftY() > 0.5 && armExtensionPotentiometer.get() > 0.05) {
        armExtension.set(-1);
      } else {
        armExtension.set(0);
      }

      //open is circle
      
      if (ps2.getCircleButton()) {
        clawSolenoid1.set(DoubleSolenoid.Value.kForward);
        clawSolenoid2.set(DoubleSolenoid.Value.kForward);
        closed = false;
      } else if (ps2.getSquareButton() && !closed) {
        clawSolenoid1.set(DoubleSolenoid.Value.kForward);
        clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
        closed = true;
      } else if (ps2.getTriangleButton() && !closed) {
        clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
        clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
        closed = true;
      } 

      if (ps2.getCrossButtonPressed() && topcam == false) {
        cameraSelection.setString(topCamera.getName());
        topcam = true;
      } else if (ps2.getCrossButtonPressed() && topcam == true) {
        cameraSelection.setString(bottomCamera.getName());
        topcam = false;
      }

      if (ps2.getR3Button()) {
        PneumaticsCompressor.enableDigital();
      } else if (ps2.getL3Button()) {
        PneumaticsCompressor.disable();
      }

      if (ps2.getRightY() > 0.5) {
        servoAngle -= 0.01;
      } else if (ps2.getRightY() < -0.5) {
        servoAngle += 0.01;
      } 
      if (servoAngle > 0.6) {
        servoAngle = 0.6;
      }
      if (servoAngle < 0) {
        servoAngle = 0;
      }
      cameraServo.set(servoAngle);

      if (ps2.getOptionsButtonPressed()) {
        scoreConeTop();
      }

      if (ps2.getShareButtonPressed()) {
        balanceOnTeeter(100000, false, true, false);
      }
    }

    //Drivers MUST have the robot completely stopped sometime during the last 15 seconds for the robot to auto teeter. They also must stop it in front of the docking station
    if(Timer.getFPGATimestamp() - autoStart >= 135) {
      balanceOnTeeter(149.9, false, true, false);
    }

    //Reading measurements
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
    SmartDashboard.putNumber("arm extension", armExtensionPotentiometer.get());

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
    armActuator.set(0);
    armExtension.set(0);
    PneumaticsControl.disableCompressor();
    clawSolenoid1.set(DoubleSolenoid.Value.kOff);
    clawSolenoid2.set(DoubleSolenoid.Value.kOff);
    // intake.set(ControlMode.PercentOutput, 0);
  }

  public void scoreConeTop() {
    targetArmAngle = Constants.topConeAngle;
    targetExtensionLength = Constants.topArmExtention;

    armAngleThread.start();
    armExtensionThread.start();

    if(Constants.blueAlliance) {
      while(Math.abs(0.0 - SmartDashboard.getNumber("Angle", 0.0)) > Constants.angleTolerance && Math.abs(1.38 - SmartDashboard.getNumber("Position X", 0.0)) > Constants.xPositionTolerance) {
        try {
          Thread.sleep(100);
        } catch(InterruptedException e) {

        }
      }
    } else {
      while(Math.abs(180.0 - SmartDashboard.getNumber("Angle", 0.0)) > Constants.angleTolerance && Math.abs((16.54 - 1.38) - SmartDashboard.getNumber("Position X", 0.0)) > Constants.xPositionTolerance) {
        try {
          Thread.sleep(100);
        } catch(InterruptedException e) {

        }
      }
    }

    while(Math.abs(armPotentiometer.get() - targetArmAngle) > Constants.armAngleTolerance && Math.abs(armExtensionPotentiometer.get() - targetExtensionLength) > Constants.armLengthTolerance) {
      try {
        Thread.sleep(100);
      } catch(InterruptedException e) {

      }
    }

    clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void scoreCubeTop() {
    //Code for scoring a cube
  }

  public void scoreCone() {
    //Code for scoring a cube
  }

  public void scoreCubeBot() {
    //Code for scoring a cube
  }
  
  public void grabCone() {
    //I'm Kevin and I'm difficult
  }
  public void grabCube() {
    //Oh boohoo I'm going to cry
  }

  public void balanceOnTeeter() {
    
  }
  
  //PRECONDITION: The robot IS NOT moving and is in front of the teeter totter
  public void balanceOnTeeter(double time, boolean left, boolean middle, boolean right) {
    double y1 = Constants.teeterCornersY[1];
    double x1 = Constants.teeterCornersX[1];
    double y2 = Constants.teeterCornersY[2];
    double x2 = Constants.teeterCornersX[2];
    double y3 = Constants.teeterCornersY[3];
    double x3 = Constants.teeterCornersX[3];
    double y0 = Constants.teeterCornersY[0];
    double x0 = Constants.teeterCornersX[0];
    
    if(left) {
      if(Math.sqrt(Math.pow(y1 - SmartDashboard.getNumber("Position Y", 0.0), 2) + Math.pow(x1 - SmartDashboard.getNumber("Position X", 0.0), 2)) < 
      Math.sqrt(Math.pow(y2 - SmartDashboard.getNumber("Position Y", 0.0), 2) + Math.pow(x2 - SmartDashboard.getNumber("Position X", 0.0), 2))) {
        goTo(y1, x1, 180.0);
      } else {
        goTo(y2, x2, 0.0);
      }

    } else if(middle) {
      if(x0 - SmartDashboard.getNumber("Position X", 0.0) < x3 - SmartDashboard.getNumber("Position X", 0.0)) {
        goTo(x0, ((y1 - y0) / 2.0) + y0, 180.0);
      } else {
        goTo(x3, ((y1 - y0) / 2.0) + y0, 0.0);
      }

    } else if(right) {
      if(Math.sqrt(Math.pow(y3 - SmartDashboard.getNumber("Position Y", 0.0), 2) + Math.pow(x3 - SmartDashboard.getNumber("Position X", 0.0), 2)) < 
      Math.sqrt(Math.pow(y0 - SmartDashboard.getNumber("Position Y", 0.0), 2) + Math.pow(x0 - SmartDashboard.getNumber("Position X", 0.0), 2))) {
        goTo(y3, x3, 0.0);
      } else {
        goTo(y0, x0, 180.0);
      }

    }

    while(Timer.getFPGATimestamp() - autoStart < time) {
      goTo(Constants.teeterPositionX, SmartDashboard.getNumber("Position Y", 0.0), SmartDashboard.getNumber("Angle", 0.0));
    }
  }

  public void goTo(double newPositionX, double newPositionY, double newAngle) {
    if(isOB(newPositionX, newPositionY)) {
      return;
    }

    double slope = (newPositionX - SmartDashboard.getNumber("Position X", 0.0)) / (newPositionY - SmartDashboard.getNumber("Position Y", 0.0));
    double tempX = SmartDashboard.getNumber("Position X", 0.0);
    double tempY = SmartDashboard.getNumber("Position Y", 0.0);

    ArrayList<Double> curvePointsY = new ArrayList<Double>();
    ArrayList<Double> curvePointsX = new ArrayList<Double>();

    for(int i = 0; i < Constants.OBY.length; i++) {
      if(tempY < Constants.OBY[i][0] && newPositionY > Constants.OBY[i][1]) {
        double startX = (Constants.OBY[i][0] - tempY) * slope + tempX;
        double endX = (Constants.OBY[i][1] - tempY) * slope + tempX;

        if((startX >= Constants.OBX[i][0] && startX <= Constants.OBX[i][1]) || (endX >= Constants.OBX[i][0] && endX <= Constants.OBX[i][1])) {
          curvePointsY.add(tempY);
          curvePointsX.add(tempX);

          if(newPositionY < tempY) {
            curvePointsY.add(Constants.OBY[i][1]);
            tempY = Constants.OBY[i][1];

            if(Constants.OBY[i][0] > newPositionY) {
              curvePointsY.add(Constants.OBY[i][0]);
              tempY = Constants.OBY[i][0];
            }
          } else {
            curvePointsY.add(Constants.OBY[i][0]);
            tempY = Constants.OBY[i][0];

            if(Constants.OBY[i][1] < newPositionY) {
              curvePointsY.add(Constants.OBY[i][1]);
              tempY = Constants.OBY[i][1];
            }
          }
          if(isOB(Constants.OBY[i][0], Constants.OBX[i][0] - 1.0)) {
            curvePointsX.add(Constants.OBX[i][1]);
            tempX = Constants.OBX[i][1];

          } else if(isOB(Constants.OBY[i][0], Constants.OBX[i][1] - 1.0)) {
            curvePointsX.add(Constants.OBX[i][0]);
            tempX = Constants.OBX[i][0];

          } else if(Math.abs(startX - Constants.OBX[i][0]) < Math.abs(startX - Constants.OBX[i][1])) {
            curvePointsX.add(Constants.OBX[i][0]);
            tempX = Constants.OBX[i][0];

          } else {
            curvePointsX.add(Constants.OBX[i][1]);
            tempX = Constants.OBX[i][1];
          }
          
          if(curvePointsX.size() < curvePointsY.size()) {
            curvePointsX.add(curvePointsX.get(curvePointsX.size() - 1));
          }
        }
      }
    }

    curvePointsY.add(newPositionY);
    curvePointsX.add(newPositionX);

    while(curvePointsY.size() > 0) {
      double targetY = curvePointsY.remove(0);
      double targetX = curvePointsX.remove(0);
      double yLine = SmartDashboard.getNumber("Position Y", 0.0);

      while(Math.abs(targetX - SmartDashboard.getNumber("Position X", 0.0)) > Constants.xPositionTolerance && Math.abs(targetY - SmartDashboard.getNumber("Position Y", 0.0)) > Constants.yPositionTolerance) {
        while((Math.abs(targetX - SmartDashboard.getNumber("Position X", 0.0)) <= Constants.xPositionTolerance && Math.abs(targetY - SmartDashboard.getNumber("Position Y", 0.0)) > Constants.yPositionTolerance)
        || Math.abs(yLine - SmartDashboard.getNumber("Position Y", 0.0)) > Constants.yLineTolerance) {
          if(targetX - SmartDashboard.getNumber("Position X", 0.0) <= Constants.xPositionTolerance) {
            if(Math.abs((SmartDashboard.getNumber("Position Y", 0.0) < targetY ? 270.0: 90.0) - SmartDashboard.getNumber("Angle", 0.0)) >= Constants.angleTolerance) {
              orient(SmartDashboard.getNumber("Position Y", 0.0) < targetY ? 270.0: 90.0);
            }
          } else {
            if(Math.abs((SmartDashboard.getNumber("Position Y", 0.0) < yLine ? 270.0 : 90.0) - SmartDashboard.getNumber("Angle", 0.0)) >= Constants.angleTolerance) {
              orient(SmartDashboard.getNumber("Position Y", 0.0) < yLine ? 270.0 : 90.0);
            }
          }

          driveLeftA.set(1);
          driveRightA.set(1);

          try {
            Thread.sleep(50);
          } catch(InterruptedException e) {

          }
        }

        if(Math.abs((targetX > SmartDashboard.getNumber("Position X", 0.0) ? 0.0 : 180.0) - SmartDashboard.getNumber("Angle", 0.0)) > Constants.angleTolerance) {
          orient(targetX > SmartDashboard.getNumber("Position X", 0.0) ? 0.0 : 180.0);
        }

        driveLeftA.set(1);
        driveRightA.set(1);

        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {

        }
      }
    }
    
    orient(newAngle);
    //Hopefully that's all
  }

  public void orient(double targetAngle) {
      boolean less180 = false;

      if(targetAngle - SmartDashboard.getNumber("Angle", 0.0) < 180) {
        less180 = true;
        driveLeftA.set(-1);
        driveRightA.set(1);

      } else {
        driveLeftA.set(1);
        driveRightA.set(-1);
      }

      //Might have to change to quarter-circle calculations instead
      while(Math.abs(targetAngle - (SmartDashboard.getNumber("Angle", 0.0) + (((SmartDashboard.getNumber("Angular Velocity", 0.0) / SmartDashboard.getNumber("Angular Acceleration", 0.0)) * SmartDashboard.getNumber("Angular Velocity", 0.0)) / 2.0))) > Constants.angleTolerance) {
        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {

        }
      }

      //Might need to be 0 and -1
      if(less180) {
        driveLeftA.set(1);
        driveRightA.set(-1);

      } else {
        driveLeftA.set(-1);
        driveRightA.set(1);
      }

      while(Math.abs(targetAngle - SmartDashboard.getNumber("Angle", 0.0)) <= Constants.angleTolerance) {
        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {

        }
      }

      driveLeftA.set(0);
      driveRightA.set(0);
  }

  public boolean isOB(double aPositionX, double aPositionY) {
    for(int i = 0; i < Constants.OBX.length; i++) {
      if(aPositionX >= Constants.OBX[i][0] && aPositionX <= Constants.OBX[i][1] && aPositionY >= Constants.OBY[i][0] && aPositionY <= Constants.OBY[i][1]) {
        return true;
      }
    }

    return false;
  }
}