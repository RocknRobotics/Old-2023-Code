/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/
package frc.robot;

//Java stuff
import java.io.*;
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

  //Motor Encoder
  RelativeEncoder leftAEncoder = driveLeftA.getEncoder();

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
  AHRS accelerometer = new AHRS(Port.kMXP, (byte) 4);
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

  Thread accelThread;

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

  //Variables
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
    PneumaticsControl.enableCompressorAnalog(120, 130);
    PneumaticsCompressor.enableAnalog(120, 130);
    PneumaticsCompressor.enableDigital();

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
    SmartDashboard.putString("DRIVE CONTROL", "OFF");
    SmartDashboard.putString("ARM CONTROL", "OFF");

    // accelerometers
    SmartDashboard.putNumber("accelerometer X", accelerometer.getWorldLinearAccelX());
    SmartDashboard.putNumber("accelerometer Z", accelerometer.getWorldLinearAccelZ());
    SmartDashboard.putNumber("accelerometer Y", accelerometer.getWorldLinearAccelY());
    SmartDashboard.putNumber("Velocity X (left/right)", accelerometer.getVelocityX());
    SmartDashboard.putNumber("Velocity Z (Forwards/Backwards)", accelerometer.getVelocityZ());
    SmartDashboard.putNumber("Velocity Y (Up/Down)", accelerometer.getVelocityY());
    SmartDashboard.putNumber("Position X (left/right)", accelerometer.getDisplacementX());
    SmartDashboard.putNumber("Position Z (Forwards/Backwards)", accelerometer.getDisplacementZ());
    SmartDashboard.putNumber("Position Y (Up/Down)", accelerometer.getDisplacementY());
    SmartDashboard.putNumber("Angular Acceleration", angularAccel);
    SmartDashboard.putNumber("Angular Velocity", angularVelocity);
    SmartDashboard.putNumber("Angle", angle);
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

    accelThread = new Thread(() -> {
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

    //Low priority thread; minor increases in time between running shouldn't affect it too much
    accelThread.setPriority(Thread.MIN_PRIORITY);
    accelThread.setDaemon(true);
    accelThread.start();

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
    //Start Compressor
    PneumaticsCompressor.enableAnalog(100, 120);
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();

    rampUp(0.4, 0.4,0.025,50);
    rampDown(-0.4, -0.4, 0.025, 50);
    rampUp(-0.1, -0.1, 0.025, 50);

    driveLeftA.set(0);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(0);
    driveRightB.follow(driveRightA); 

    rampUp(0.5, 0.5, 0.025, 50);

    try{
      Thread.sleep(750);
    } catch(InterruptedException e){

    }

    rampDown(0, 0, 0.025, 50);

    driveLeftA.set(0);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(0);
    driveRightB.follow(driveRightA); 

    armActuator.set(1);
    try{
      Thread.sleep(3000);
    } catch(InterruptedException e){

    }

    //Stop Arm
    armActuator.set(0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    PneumaticsCompressor.enableDigital();
    PneumaticsCompressor.enableAnalog(100, 120);
    SmartDashboard.putString("DRIVE CONTROL", "ON");
    SmartDashboard.putString("ARM CONTROL", "ON");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    //disable(no movement and brakes) and enable(movement and maybe doesnt brake)
    if (ps1.getPSButtonPressed()) {
      stopped1 = !stopped1;
    }

    if (ps2.getPSButtonPressed()) {
      stopped2 = !stopped2;
    }

    // Set up arcade steer
    double forward = 0;
    double turn = 0;

    // PS4
    if (!stopped1) {
      if(ps1.getTriangleButtonReleased()){
        fast = !fast;
      }
      if (!fast) {
        // regular mode & left trigger backward & right trigger foward
        forward = (-1 * (ps1.getL2Axis() * 0.1)) + (ps1.getR2Axis() * 0.1);
        if (Math.abs(ps1.getRightX()) > .15 || Math.abs(ps1.getLeftX()) > .15) {
          turn = Math.min((ps1.getRightX() + ps1.getLeftX()), 1)/ 8;// right/left stick steer x-axis
          if(turn < 0) turn = Math.max(turn, -0.125);
        }
      } else {
        // fast mode & left trigger backward & right trigger foward
        forward = (-1 * (ps1.getL2Axis()) * 0.5) + (ps1.getR2Axis() * 0.5);
        if (Math.abs(ps1.getRightX()) > .15 || Math.abs(ps1.getLeftX()) > .15) {
          turn = Math.min(ps1.getRightX() + ps1.getLeftX(), 1) * 0.35;// right/left stick steer x-axis
          if(turn < 0) turn = Math.max(turn, -.35);
        }
      }
      double driveLeftPower = (forward + turn);
      double driveRightPower = (forward - turn);

      if (fast) {
        leftSpeed = leftSpeed + (driveLeftPower - leftSpeed) / 10;
        rightSpeed = rightSpeed + (driveRightPower - rightSpeed) / 10;
      } else {
        leftSpeed = leftSpeed + (driveLeftPower - leftSpeed) / 5;
        rightSpeed = rightSpeed + (driveRightPower - rightSpeed) / 5;
      }

      driveLeftA.set(leftSpeed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(rightSpeed);
      driveRightB.follow(driveRightA);

      SmartDashboard.putString("DRIVE CONTROL", "ON");
    } else {
      SmartDashboard.putString("DRIVE CONTROL", "OFF");
      //Im not sure if this is going to be able to drive when we go again
      driveLeftA.setIdleMode(IdleMode.kBrake);
      driveLeftB.setIdleMode(IdleMode.kBrake);
      driveRightA.setIdleMode(IdleMode.kBrake);
      driveRightB.setIdleMode(IdleMode.kBrake);
    }
    //arm testing
    //highest arm angle is 0.442
    //lowest is 
    if (!stopped2) {
      if (ps2.getRightY() < -0.5) {
        armActuator.set(1);
      } else if (ps2.getRightY() > 0.5) {
        armActuator.set(-1);
      } else {
        armActuator.set(0);
      }

      if (ps2.getLeftY() < -0.5 && armExtensionPotentiometer.get() < 0.72) {
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
        //PneumaticsCompressor.enableDigital();
      } else if (ps2.getL3Button()) {
        PneumaticsCompressor.disable();
      }

      SmartDashboard.putString("ARM CONTROL", "ON");
    } else {
      SmartDashboard.putString("ARM CONTROL", "OFF");
    } 

    //Reading measurements
    SmartDashboard.putNumber("accelerometer X", accelX);
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
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
    SmartDashboard.putNumber("arm extension", armExtensionPotentiometer.get());
  }

  /* 
  private FileWriter recorder = Constants.recorder();
  public void practiceInit()
  {}

  public void practicePeriodic()
  {
    if (recorder == null)
    {
      return;
    }
    if (ps1.getCircleButton())
    {
      recorder.write("C" + true);
    }
  }
*/
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
    SmartDashboard.putString("DRIVE CONTROL", "OFF");
    SmartDashboard.putString("ARM CONTROL", "OFF");
  }

  public void scoreConeTop() {
    targetArmAngle = Constants.topConeAngle;
    targetExtensionLength = Constants.topArmExtention;

    armAngleThread.start();
    armExtensionThread.start();

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

  //Recommended increment value of 0.025 with a sleep of 25
  //Absolute value of target speed should equal each other
  //Absolute value of current speeds should equal each other
  public void rampUp(double targetSpeedL, double targetSpeedR, double increment, long sleepyTime) {
    double rightSpeed = 0;
    double leftSpeed = 0;
    for(; leftSpeed < targetSpeedL || rightSpeed < targetSpeedR; 
      leftSpeed += increment, rightSpeed += increment){
          if(rightSpeed >= targetSpeedR) rightSpeed = targetSpeedR;
          if(leftSpeed >= targetSpeedL) leftSpeed = targetSpeedL;
          
          driveLeftA.set(leftSpeed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(rightSpeed);
          driveRightB.follow(driveRightA);
          System.out.println("left Speed: "+leftSpeed);
          System.out.println("right Speed: "+rightSpeed);
          try {Thread.sleep(sleepyTime);} catch(InterruptedException e) {}
      }
  }

  public void rampDown(double targetSpeedL, double targetSpeedR, double increment, long sleepyTime) {
    double rightSpeed = 0;
    double leftSpeed = 0;
    for(; leftSpeed > targetSpeedL || rightSpeed > targetSpeedR; 
      leftSpeed -= increment, rightSpeed -= increment){
          if(rightSpeed < targetSpeedR) rightSpeed = targetSpeedR;
          if(leftSpeed < targetSpeedL) leftSpeed = targetSpeedL;
          
          driveLeftA.set(leftSpeed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(rightSpeed);
          driveRightB.follow(driveRightA);

          try {Thread.sleep(sleepyTime);} catch(InterruptedException e) {}
      }
    }
}
