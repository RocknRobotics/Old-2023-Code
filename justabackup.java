
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
import com.revrobotics.RelativeEncoder;

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

  RelativeEncoder leftAEncoder = driveLeftA.getEncoder();

  //Linear actuator
  CANSparkMax armActuator = new CANSparkMax(7, MotorType.kBrushed);

  //Arm Extension 
  static CANSparkMax armExtension = new CANSparkMax(8, MotorType.kBrushed);

  //Camera Servo
  Servo cameraServo = new Servo(0);

  //Pneumatics
  PneumaticsControlModule PneumaticsControl = new PneumaticsControlModule();
  Compressor PneumaticsCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid clawSolenoid1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid clawSolenoid2 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
  
  double servoAngle = 0;.0

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(1);
  public static AnalogPotentiometer armExtensionPotentiometer = new AnalogPotentiometer(0);

  //Arm threads
  boolean armRunning = false;
  Thread armAngleThread;
  double targetArmAngle = 0.0;
  boolean armExtending = false;
  Thread armExtensionThread;
  double targetExtensionLength = 0.0;

  //Controller
  PS4Controller ps1 = new PS4Controller(0);
  PS4Controller ps2 = new PS4Controller(1);

  //Camera
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
  Thread iPromiseThisWontBreakAnything;
  double experimentalVelocityConstant = 0.0;
  boolean stopped1 = false;
  boolean stopped2 = false;

 //Station (starting position) and cone/cube
  int station = -1;

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
    bottomCamera.setFPS(15);
    //Need to determine if this resolution will work well
    bottomCamera.setResolution(600, 400);
    //I am well aware this is a very bad way of doing this, however I know this will work so that's why I'm using it
    //I'll probably refactor after the competition
    SmartDashboard.putNumber("Top Width", 600);
    SmartDashboard.putNumber("Top Height", 400);
    //NEED to find experimental values for both cameras
    SmartDashboard.putNumber("Top Pixel Angle X", 0.0);
    //The angle of the camera in relation to the arm such that facing the same direction is 0.0 and facing directly opposite is 180.0
    SmartDashboard.putNumber("Top Camera Angle", 0.0);
    MjpegServer topServer = CameraServer.addServer("Top Stream", 1181);

    topCamera = CameraServer.startAutomaticCapture("Top", 1);
    topCamera.setFPS(15);
    bottomCamera.setResolution(600, 400);
    SmartDashboard.putNumber("Bottom Width", 600);
    SmartDashboard.putNumber("Bottom Height", 400);
    SmartDashboard.putNumber("Bottom Pixel Angle X", 0.0);
    SmartDashboard.putNumber("Bottom Camera Angle", 0.0);
    MjpegServer bottomServer = CameraServer.addServer("Bottom Stream", 1182);

    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    cameraSelection.setString(bottomCamera.getName());
    topcam = false;

    // add a thing on the dashboard to turn off auto if needed
    // SmartDashboard.put
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
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

    armAngleThread = new Thread(() -> {
      armRunning = true;

      while(Math.abs(armPotentiometer.get() - targetArmAngle) > Constants.armAngleTolerance) {
        if(armPotentiometer.get() < targetArmAngle) {
          armActuator.set(-1);
        } else {
          armActuator.set(1);
        }

        try {
          Thread.sleep(250);
        } catch(InterruptedException e) {
          
        }
      }

      armActuator.set(0);
      armRunning = false;
    });
    armAngleThread.setPriority(Thread.MIN_PRIORITY);
    armAngleThread.setDaemon(true);

    armExtensionThread = new Thread(() -> {
      armExtending = true;

      while(Math.abs(armExtension.get() - targetExtensionLength) > Constants.armLengthTolerance) {
        if(armExtension.get() < targetExtensionLength) {
          armExtension.set(-1);
        } else {
          armExtension.set(1);
        }

        try {
          Thread.sleep(250);
        } catch(InterruptedException e) {
          
        }
      }

      armExtension.set(0);
      armExtending = false;
    });
    armExtensionThread.setPriority(Thread.MIN_PRIORITY);
    armExtensionThread.setDaemon(true);

    iPromiseThisWontBreakAnything = new Thread(() -> {
      while(true) {
        if(experimentalVelocityConstant != 0.0) {
          SmartDashboard.putNumber("Left Speed", driveLeftA.get());
          SmartDashboard.putNumber("Right Speed", driveRightA.get());
        }

        try {
          Thread.sleep(50);
        } catch(InterruptedException e) {

        }
      }
    });
    iPromiseThisWontBreakAnything.setPriority(Thread.MAX_PRIORITY);
    iPromiseThisWontBreakAnything.setDaemon(true);
    iPromiseThisWontBreakAnything.start();
  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto

    /*targetArmAngle = 0.5;
    armAngleThread.start();
    targetExtensionLength = 0.755;
    armExtensionThread.start();*/

    //clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    //clawSolenoid2.set(DoubleSolenoid.Value.kReverse);

    //rampUp(0.4, 0.4, 0.025, 25);

    for(double speed = 0; speed <= 0.4; speed += 0.025) {
      if(speed == 0.025) {
        double currTime = Timer.getFPGATimestamp();
        leftAEncoder.setPosition(0.0);
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(25);
        } catch(InterruptedException e) {

        }

        experimentalVelocityConstant = 1.0 / ((leftAEncoder.getPosition() * leftAEncoder.getPositionConversionFactor() / (Timer.getFPGATimestamp() - currTime)) * speed);
      } else {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);

        try {
          Thread.sleep(25);
        } catch(InterruptedException e) {

        }
      }
    }

    leftSpeed = -0.1;
    rightSpeed = -0.1;

    rampUp(-0.3, -0.3, 0.025, 25);

    rampUp(-0.1, -0.1, 0.025, 25);

    driveLeftA.set(0);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(0);
    driveRightB.follow(driveRightA);

    rampUp(0.5, 0.5, 0.015, 25);

    rampUp(0, 0, 0.025, 25);

    /**for(double speed = 1; speed >= -1; speed -= 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);

      try {
        Thread.sleep(25);
      } catch(InterruptedException e) {

      }
    }

    if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 1;

      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }

      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      }

      for(double speed = 0; speed <= 1; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }

      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      }
    } else if(SmartDashboard.getBoolean("Station 2", true)) {
      station = 2;

      for(double speed = 0; speed <= -1; speed -= 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }
      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }
    } else if(SmartDashboard.getBoolean("Station 3", true)) {
      station = 3;

      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }

      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      }

      for(double speed = 0; speed <= 1; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {

        }
      }

      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {

          }
        }
      }
    }

    for(double speed = 0; speed <= 1; speed += 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);

      try {
        Thread.sleep(10);
      } catch(InterruptedException e) {

      }
    }
    for(double speed = 1; speed >= 0; speed -= 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);

      try {
        Thread.sleep(10);
      } catch(InterruptedException e) {
        
      }
    }

    driveLeftA.setInverted(false);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(false);
    driveLeftB.burnFlash();
    driveRightA.setInverted(true);
    driveRightA.burnFlash();
    driveRightB.setInverted(true);
    driveRightB.burnFlash();*/
    /*if(!SmartDashboard.getBoolean("Go For Auto", true)) {
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
  //balanceOnTeeter(14.9, false, true, false);*/
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      Thread.sleep(100);
    } catch(InterruptedException e) {

    }
    /*if (goForAuto) {

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
    }*/
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    driveLeftA.setInverted(false);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(false);
    driveLeftB.burnFlash();
    driveRightA.setInverted(true);
    driveRightA.burnFlash();
    driveRightB.setInverted(true);
    driveRightB.burnFlash();
    //accelerometer.resetDisplacement();
    //Positions.startPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //accelThread.start();
    //disable and enable]

    if (ps1.getPSButtonPressed()) {
      stopped1 = !stopped1;
    }

    if (ps2.getPSButtonPressed()) {
      stopped2 = !stopped2;
    }

    //If driver controller is stopped then it will start auto driving (will not affect arm controller directly)
    //But auto drive code could conflict with arm controls
    if(stopped1) {
      SmartDashboard.putBoolean("Auto Drive", true);

      try {
        Thread.sleep(100);
      } catch(InterruptedException e) {

      }

      if(SmartDashboard.getBoolean("Auto Drive", false)) {
        targetArmAngle = SmartDashboard.getNumber("Target Actuator", armPotentiometer.get());
        if(!armRunning) {
          armAngleThread.start();
        }
        targetExtensionLength = SmartDashboard.getNumber("Target Potentiometer", armExtensionPotentiometer.get());
        if(!armExtending) {
          armExtensionThread.start();
        }

        orient(SmartDashboard.getNumber("Target Angle", 0.0));
        drive(SmartDashboard.getNumber("Target Position X", 0.0), SmartDashboard.getNumber("Target Position Y", 0.0));
      }
    } else {
      SmartDashboard.putBoolean("Auto Drive", false);
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
        armActuator.set(ps2.getL2Axis());
      } else if (ps2.getR2Axis() > 0.5) {
        armActuator.set(-1 * ps2.getR2Axis());
      } else {
        armActuator.set(0);
      }

      if (ps2.getLeftY() < -0.5 && armExtensionPotentiometer.get() < 0.645) {
        armExtension.set(1);
      } else if (ps2.getLeftY() > 0.5 ) {
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
      /*
      if (ps2.getCrossButtonPressed() && topcam == false) {
        cameraSelection.setString(topCamera.getName());
        topcam = true;
      } else if (ps2.getCrossButtonPressed() && topcam == true) {
        cameraSelection.setString(bottomCamera.getName());
        topcam = false;
      }
      */

      if (ps2.getR3Button()) {
        PneumaticsCompressor.start();
      } else if (ps2.getL3Button()) {
        PneumaticsCompressor.stop();
      }

      if (ps1.getRightY() > 0.5) {
        servoAngle -= 0.01;
      } else if (ps1.getRightY() < -0.5) {
        servoAngle += 0.01;
      } 
      if (servoAngle > 0.6) {
        servoAngle = 0.6;
      }
      if (servoAngle < 0) {
        servoAngle = 0;
      }
      cameraServo.set(servoAngle);

    /*
      if (ps2.getOptionsButtonPressed()) {
        scoreConeTop();
      }

      if (ps2.getShareButtonPressed()) {
        balanceOnTeeter(100000, false, true, false);
      }
      */
    }

    //Drivers MUST have the robot completely stopped sometime during the last 15 seconds for the robot to auto teeter. They also must stop it in front of the docking station
    if(Timer.getFPGATimestamp() - autoStart >= 135) {
      //balanceOnTeeter(149.9, false, true, false);
    }

    /*positionX += accelerometer.getVelocityX() * accelTime;
    positionY += accelerometer.getVelocityY() * accelTime;
    positionZ += accelerometer.getVelocityZ() * accelTime;

    double prevAngle = angle;
    double prevVelocity = angularVelocity;
    angle = accelerometer.getYaw() < 0 ? accelerometer.getYaw() + 360.0 : accelerometer.getYaw();
    angularVelocity = (angle - prevAngle) / (Timer.getFPGATimestamp() - accelTime);
    angularAccel = (prevVelocity - angularVelocity) / (Timer.getFPGATimestamp() - accelTime);
    accelTime = Timer.getFPGATimestamp();*/

    //Reading measurements
    /*SmartDashboard.putNumber("accelerometer X", accelX);
    SmartDashboard.putNumber("accelerometer Z", accelZ);
    SmartDashboard.putNumber("accelerometer Y", accelY);
    SmartDashboard.putNumber("Velocity X (left/right)", velocityX);
    SmartDashboard.putNumber("Velocity Z (Forwards/Backwards)", velocityZ);
    SmartDashboard.putNumber("Velocity Y (Up/Down)", velocityY);
    SmartDashboard.putNumber("Position X (left/right)", positionX);
    SmartDashboard.putNumber("Position Z (Forwards/Backwards)", positionZ);
    SmartDashboard.putNumber("Position Y (Up/Down)", positionY);
    SmartDashboard.putNumber("Angular Acceleration", angularAccel);
    SmartDashboard.putNumber("Angular Velocity", angularVelocity);
    SmartDashboard.putNumber("Angle", angle);*/
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

  public void orient(double targetAngle) {
    double speed = 0;
    double totalTime = 0;

    while(Math.abs((SmartDashboard.getNumber("Angle", 0.0) + (totalTime * SmartDashboard.getNumber("Angular Velocity", 0.0))) - targetAngle) > Constants.angleTolerance) {
      if(!SmartDashboard.getBoolean("Auto Drive", false)) {
        return;
      }
      speed += 0.025;
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1 * speed);
      driveRightB.follow(driveRightA);

      try {
        Thread.sleep(25);
      } catch(InterruptedException e) {

      }

      totalTime += 0.025;
    }

    while(speed > 0) {
      if(!SmartDashboard.getBoolean("Auto Drive", false)) {
        return;
      }
      speed -= 0.025;
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1 * speed);
      driveRightB.follow(driveRightA);

      try {
        Thread.sleep(25);
      } catch(InterruptedException e) {

      }
    }

    //ASSUMES that it is already angled correctly and that the new positions don't cross out of bounds
    //Should always be driving backwards to avoid hitting other robots with the arm
    public void drive(double newPositionX, double newPositionY) {
      double speed = 0.0;
      double totalTime = 0.0;

      if(newPositionX == SmartDashboard.getNumber("Position X", 0.0)) {
        //Y changes
        while(Math.abs((SmartDashboard.getNumber("Position Y", 0.0) + (totalTime * SmartDashboard.getNumber("Velocity Y", 0.0))) - newPositionY) > Constants.positionTolerance) {
          if(!SmartDashboard.getBoolean("Auto Drive", false)) {
            return;
          }
          speed -= 0.025;
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(25);
          } catch(InterruptedException e) {

          }

          totalTime += 0.025;
        }

        while(speed < 0) {
          if(!SmartDashboard.getBoolean("Auto Drive", false)) {
            return;
          }
          speed += 0.025;
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(25);
          } catch(InterruptedException e) {

          }
        }

      } else {
        //X changes
        while(Math.abs((SmartDashboard.getNumber("Position X", 0.0) + (totalTime * SmartDashboard.getNumber("Velocity X", 0.0))) - newPositionY) > Constants.positionTolerance) {
          speed -= 0.025;
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(25);
          } catch(InterruptedException e) {

          }

          totalTime += 0.025;
        }

        while(speed < 0) {
          speed += 0.025;
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(25);
          } catch(InterruptedException e) {

          }
        }
      }
    }
  }

  /*public void scoreConeTop() {
    targetArmAngle = Constants.topConeAngle;
    targetExtensionLength = Constants.topArmExtention;

    armAngleThread.start();
    armExtensionThread.start();

    clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
  }*/

  //Recommended increment value of 0.025 with a sleep of 25
  //Absolute value of target speed should equal each other
  //Absolute value of current speeds should equal each other
  public void rampUp(double targetSpeedL, double targetSpeedR, double increment, long sleepyTime) {
    if(targetSpeedL < leftSpeed) {
      if(targetSpeedR < rightSpeed) {
        for(double speed = leftSpeed; speed >= targetSpeedL; speed -= increment) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(sleepyTime);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = leftSpeed; speed >= targetSpeedL; speed -= increment) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(sleepyTime);
          } catch(InterruptedException e) {
            
          }
        }
      }
    } else {
      if(targetSpeedR > rightSpeed) {
        for(double speed = leftSpeed; speed <= targetSpeedL; speed += increment) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(sleepyTime);
          } catch(InterruptedException e) {

          }
        }
      } else {
        for(double speed = leftSpeed; speed <= targetSpeedL; speed += increment) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);

          try {
            Thread.sleep(sleepyTime);
          } catch(InterruptedException e) {
            
          }
        }
      }
    }
  }
  
  /*public void scoreCubeTop() {
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
      if(Math.sqrt(Math.pow(y1 - positionY, 2) + Math.pow(x1 - positionX, 2)) < 
      Math.sqrt(Math.pow(y2 - positionY, 2) + Math.pow(x2 - positionX, 2))) {
        goTo(y1, x1, 180.0);
      } else {
        goTo(y2, x2, 0.0);
      }

    } else if(middle) {
      if(x0 - positionX < x3 - positionX) {
        goTo(x0, ((y1 - y0) / 2.0) + y0, 180.0);
      } else {
        goTo(x3, ((y1 - y0) / 2.0) + y0, 0.0);
      }

    } else if(right) {
      if(Math.sqrt(Math.pow(y3 - positionY, 2) + Math.pow(x3 - positionX, 2)) < 
      Math.sqrt(Math.pow(y0 - positionY, 2) + Math.pow(x0 - positionX, 2))) {
        goTo(y3, x3, 0.0);
      } else {
        goTo(y0, x0, 180.0);
      }

    }

    while(Timer.getFPGATimestamp() - autoStart < time) {
      goTo(Constants.teeterPositionX, positionY, angle);
    }
  }

  public void goTo(double newPositionX, double newPositionY, double newAngle) {
    if(isOB(newPositionX, newPositionY)) {
      return;
    }
    double slope = (newPositionX - positionX) / (newPositionY - positionY);
    double tempX = positionX;
    double tempY = positionY;
    ArrayList<Double> curvePointsY = new ArrayList<Double>();
    ArrayList<Double> curvePointsX = new ArrayList<Double>();
    for(int i = 0; i < Constants.OBY.length; i++) {
      if(tempY < Constants.OBY[i][0] && newPositionY > Constants.OBY[i][1]) {
        double startX = (Constants.OBY[i][0] - tempY) * slope + tempX;
        double endX = (Constants.OBY[i][1] - tempY) * slope + tempX;
        if((startX >= Constants.OBX[i][0] && startX <= Constants.OBX[i][1]) || (endX >= Constants.OBX[i][0] && endX <= Constants.OBX[i][1])) {
          curvePointsY.add(tempY);
          curvePointsX.add(tempX);
          if(newPositionY < positionY) {
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
      double currTime = Timer.getFPGATimestamp();
      double targetY = curvePointsY.remove(0);
      double targetX = curvePointsX.remove(0);
      double yLine = positionY;
      while(Math.abs(targetX - positionX) > Constants.xPositionTolerance && Math.abs(targetY - positionY) > Constants.yPositionTolerance) {
        while((Math.abs(targetX - positionX) <= Constants.xPositionTolerance && Math.abs(targetY - positionY) > Constants.yPositionTolerance)
        || Math.abs(yLine - positionZ) > Constants.yLineTolerance) {
          if(targetX - positionX <= Constants.xPositionTolerance) {
            if(Math.abs((positionY < targetY ? 270.0: 90.0) - angle) >= Constants.angleTolerance) {
              orient(positionY < targetY ? 270.0: 90.0);
            }
          } else {
            if(Math.abs((positionY < yLine ? 270.0 : 90.0) - angle) >= Constants.angleTolerance) {
              orient(positionY < yLine ? 270.0 : 90.0);
            }
          }
          driveLeftA.set(1);
          driveRightA.set(1);
        }
        if(Math.abs((targetX > positionX ? 0.0 : 180.0) - angle) > Constants.angleTolerance) {
          orient(targetX > positionX ? 0.0 : 180.0);
        }
        driveLeftA.set(1);
        driveRightA.set(1);
      }
    }
    orient(newAngle);
    //Hopefully that's all
  }

  public void orient(double targetAngle) {
      double tempAngle = angle;
      double tempTime = Timer.getFPGATimestamp();
      boolean less180 = false;
      if(targetAngle - angle < 180) {
        less180 = true;
        driveLeftA.set(-1);
        driveRightA.set(1);
      } else {
        driveLeftA.set(1);
        driveRightA.set(-1);
      }
      while((Timer.getFPGATimestamp() - tempTime) < Math.pow(10, -10)) {}
      double prevAngularVelocity = 0.0;
      double angularVelocity = (angle - tempAngle) / (Timer.getFPGATimestamp() - tempTime);
      double angularAccel = (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - tempTime);
      tempAngle = angle;
      tempTime = Timer.getFPGATimestamp();
      //Might have to change to quarter-circle calculations instead
      while(Math.abs(targetAngle - (angle + (((angularVelocity / angularAccel) * angularVelocity) / 2.0))) > Constants.angleTolerance) {
        prevAngularVelocity = angularVelocity;
        angularVelocity += (angle - tempAngle) / (Timer.getFPGATimestamp() - tempTime);
        angularAccel += (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - tempTime);
        tempAngle = angle;
        tempTime = Timer.getFPGATimestamp();
      }
      //Might need to be 0 and -1
      if(less180) {
        driveLeftA.set(1);
        driveRightA.set(-1);
      } else {
        driveLeftA.set(-1);
        driveRightA.set(1);
      }
      while(Math.abs(targetAngle - angle) <= Constants.angleTolerance) {}
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
  }*/
}
