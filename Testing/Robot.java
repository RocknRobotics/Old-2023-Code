/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/
package frc.robot;

//Java stuff
import java.text.*;

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

//Camera
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//Accelerometer
import com.kauailabs.navx.frc.AHRS;

//Potentiometer
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//timer
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Controller
import edu.wpi.first.wpilibj.PS4Controller;

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

  //Accelerometer
  AHRS Gyro = new AHRS();

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
  boolean runArm = false;
  boolean extendArm = false;

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

  //Temp speed since we have a new arm (don't want to kill someone using old speed)
  double armSpeed = 0.1;


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
    bottomCamera = CameraServer.startAutomaticCapture("Bottom", "/dev/video0");
    topCamera = CameraServer.startAutomaticCapture("Top", "/dev/video1");
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    cameraSelection.setString(bottomCamera.getName());
    topcam = false;
    bottomCamera.setFPS(10);
    topCamera.setFPS(10);
    bottomCamera.setResolution(360, 360);
    topCamera.setResolution(360, 360);

    SmartDashboard.setNetworkTableInstance(NetworkTableInstance.getDefault());
    // add a thing on the dashboard to turn off auto if needed
    // SmartDashboard.put
    SmartDashboard.putBoolean("Go For Auto", true);
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
    SmartDashboard.putString("DRIVE CONTROL", "OFF");
    SmartDashboard.putString("ARM CONTROL", "OFF");

    // accelerometers
    SmartDashboard.putNumber("Yaw", Gyro.getYaw());
    SmartDashboard.putNumber("Roll", Gyro.getRoll());
    SmartDashboard.putNumber("Pitch", Gyro.getPitch());
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
    SmartDashboard.putNumber("arm extension", armExtensionPotentiometer.get());
    SmartDashboard.putNumber("Position X", Gyro.getDisplacementX());
    SmartDashboard.putNumber("Position Y", Gyro.getDisplacementY());
    SmartDashboard.putNumber("Position Z", Gyro.getDisplacementZ());
    SmartDashboard.putNumber("Left Motor", driveLeftA.get());
    SmartDashboard.putNumber("Right Motor", driveRightA.get());
    SmartDashboard.putNumber("Velocity X", Gyro.getVelocityX());
    SmartDashboard.putNumber("Velocity Y", Gyro.getVelocityY());
    SmartDashboard.putNumber("Velocity Z", Gyro.getVelocityZ());
    SmartDashboard.putNumber("Accel X", Gyro.getWorldLinearAccelX());
    SmartDashboard.putNumber("Accel Y", Gyro.getWorldLinearAccelY());
    SmartDashboard.putNumber("Accel Z", Gyro.getWorldLinearAccelZ());
    SmartDashboard.putBoolean("Test", false);
    SmartDashboard.putNumber("Arm Extension", 0.0);
    SmartDashboard.putNumber("Arm Actuator", 0.0);
    SmartDashboard.putBoolean("Enable Digital", false);
    SmartDashboard.putBoolean("Auto Balance", false);

    //Station number---All start in front of a cone stand (for now)
    SmartDashboard.putBoolean("Edge Start", false);
    SmartDashboard.putBoolean("Center Start", false);

    armAngleThread = new Thread(() -> {
      while(true) {
        while(Math.abs(armPotentiometer.get() - targetArmAngle) > Constants.armAngleTolerance && runArm) {
          if(armPotentiometer.get() < targetArmAngle) {
            armActuator.set(-1);
          } else {
            armActuator.set(1);
          }
  
          try {
            Thread.sleep(100);
          } catch(InterruptedException e) {
            
          }
        }
  
        armActuator.set(0);
        runArm = false;
  
        try {
          Thread.sleep(500);
        } catch(InterruptedException e) {
  
        }
      }
    });
    armAngleThread.setPriority(Thread.MIN_PRIORITY);
    armAngleThread.setDaemon(true);
    armAngleThread.start();

    armExtensionThread = new Thread(() -> {
      while(true) {
        while(Math.abs(armExtension.get() - targetExtensionLength) > Constants.armLengthTolerance && extendArm) {
          if(armExtension.get() < targetExtensionLength) {
            armExtension.set(-1);
          } else {
            armExtension.set(1);
          }
  
          try {
            Thread.sleep(100);
          } catch(InterruptedException e) {
            
          }
        }
  
        armExtension.set(0);
        extendArm = false;
  
        try {
          Thread.sleep(500);
        } catch(InterruptedException e) {
  
        }
      }
    });
    armExtensionThread.setPriority(Thread.MIN_PRIORITY);
    armExtensionThread.setDaemon(true);
    armExtensionThread.start();
  }

  @Override
  public void autonomousInit() {
    //PneumaticsCompressor.enableAnalog(100, 120);
    PneumaticsCompressor.enableDigital();
    autoStart = Timer.getFPGATimestamp();

    clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    clawSolenoid2.set(DoubleSolenoid.Value.kReverse);

    armActuator.set(1);
    try {
      Thread.sleep(1000);
    } catch(InterruptedException e) {
      
    }
    armExtension.set(1);

    while(armActuator.get() - 0.427 > 0.015 && armExtensionPotentiometer.get() - 0.69 > 0.03) {
      if(armActuator.get() - 0.44 > 0.01) {
        armActuator.set(0);
      }
      if(armExtensionPotentiometer.get() - 0.7 > 0.02) {
        armExtension.set(0);
      }

      try {
        Thread.sleep(50);
      } catch(InterruptedException e) {

      }
    }

    clawSolenoid1.set(DoubleSolenoid.Value.kForward);
    clawSolenoid2.set(DoubleSolenoid.Value.kForward);

    //We can get rid of this if we want the arm to stay high
    targetArmAngle = 0.1;
    targetExtensionLength = 0.1;
    runArm = true;
    extendArm = true;

    if(SmartDashboard.getBoolean("Edge Start", false)) {
      rampDown(-0.5, -0.5, 0.025, 50);

      try {
        Thread.sleep(750);
      } catch(InterruptedException e) {

      }

      rampUp(0, 0, 0.025, 50);

    } else if(SmartDashboard.getBoolean("Center Start", false)) {
      rampDown(-0.5, -0.5, 0.025, 50);

      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {

      }

      rampUp(0, 0, 0.025, 50);

      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {

      }

      teeterBalance(14.75);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    PneumaticsCompressor.enableDigital();
    //PneumaticsCompressor.enableAnalog(100, 120);
    SmartDashboard.putString("DRIVE CONTROL", "ON");
    SmartDashboard.putString("ARM CONTROL", "ON");
    Gyro.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(SmartDashboard.getBoolean("Auto Balance", false)) {
      teeterBalance(Timer.getFPGATimestamp() - autoStart + 15);
    }

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
      if (ps2.getRightY() < -0.5 /*&& armPotentiometer.get() < 0.0*/) {
        if(!(armExtensionPotentiometer.get() > 0.0 && armPotentiometer.get() < 0.0)) {
          armActuator.set(1);
        }
      } else if (ps2.getRightY() > 0.5 /*&& armPotentiometer.get() > 0.0 */) {
        armActuator.set(-1);
      } else {
        armActuator.set(0);
      }

      if (ps2.getLeftY() < -0.5 /*&& armExtensionPotentiometer.get() < 0.72*/) {
        armExtension.set(1);
      } else if (ps2.getLeftY() > 0.5 /*&& armExtensionPotentiometer.get() > 0.05*/) {
        armExtension.set(-1);
      } else {
        //armExtension.set(0.05 * armSpeed);
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

      SmartDashboard.putString("ARM CONTROL", "ON");
    } else {
      SmartDashboard.putString("ARM CONTROL", "OFF");
    } 

    //Reading measurements
    SmartDashboard.putNumber("Yaw", Gyro.getYaw());
    SmartDashboard.putNumber("Roll", Gyro.getRoll());
    SmartDashboard.putNumber("Pitch", Gyro.getPitch());
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
    SmartDashboard.putNumber("arm extension", armExtensionPotentiometer.get());
    SmartDashboard.putNumber("Position X", Gyro.getDisplacementX());
    SmartDashboard.putNumber("Position Y", Gyro.getDisplacementY());
    SmartDashboard.putNumber("Position Z", Gyro.getDisplacementZ());
    SmartDashboard.putNumber("Left Motor", driveLeftA.get());
    SmartDashboard.putNumber("Right Motor", driveRightA.get());
    SmartDashboard.putNumber("Velocity X", Gyro.getVelocityX());
    SmartDashboard.putNumber("Velocity Y", Gyro.getVelocityY());
    SmartDashboard.putNumber("Velocity Z", Gyro.getVelocityZ());
    SmartDashboard.putNumber("Accel X", Gyro.getWorldLinearAccelX());
    SmartDashboard.putNumber("Accel Y", Gyro.getWorldLinearAccelY());
    SmartDashboard.putNumber("Accel Z", Gyro.getWorldLinearAccelZ());

    if(SmartDashboard.getBoolean("Enable Digital", false)) {
      SmartDashboard.putBoolean("Enable Digital", false);
      PneumaticsCompressor.enableDigital();
    }

    /*armExtension.set(armSpeed * SmartDashboard.getNumber("Arm Extension", 0.0));
    armActuator.set(armSpeed * SmartDashboard.getNumber("Arm Actuator", 0.0));
    try {
      Thread.sleep(50);
    } catch(InterruptedException e) {

    }*/
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
    armActuator.set(0 * armSpeed);
    armExtension.set(0 * armSpeed);
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

   //Give the time since the start of the match (in seconds) for the method to balance until
    //Needs to be anywhere on teeter (literally being on the very beginning slant would probably still work)
    public void teeterBalance(double targetTime) {
      //Need to do testing to figure these out
      double kp = 1.0;
      double ki = 1.0;
      double kd = 1.0;
      double p = Gyro.getRoll();
      double i = 0.0; //Integral of p
      double currTime = Timer.getFPGATimestamp(); //Needed for integral and derivative
      double d = 0.0; //Derivative of p
      double prevP = Gyro.getRoll(); //Required for derivative

      while(Timer.getFPGATimestamp() - autoStart < targetTime) {
        //Yay calculus
        prevP = p;
        p = Gyro.getRoll();
        i += (Timer.getFPGATimestamp() - currTime) * p; //Integral is summation of p over time
        d = (p - prevP) / (Timer.getFPGATimestamp() - currTime); //Definition of a derivative
        currTime = Timer.getFPGATimestamp();

        double motorOutput = kp * p + ki * i + kd * d;
        if(Math.abs(Gyro.getYaw() - 180) <= 10 || Math.abs(Gyro.getYaw() + 180) <= 10) {
          motorOutput *= -1;
        }

        int numReduced = 0;

        while(Math.abs(motorOutput) > 0.1) {
          motorOutput /= 10.0;
          numReduced++;
        }

        SmartDashboard.putNumber("Times Reduced", numReduced);
        
        //Hopefully the pid accounts for not deciding to suddenly go from 1 to -1? We'll see
        driveLeftA.set(motorOutput);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(motorOutput);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(10); //I mean in theory this would be the only process running, but on the other hand let's not kill the roborio
        } catch(InterruptedException e) {
          
        }
      }

      //Just to be safe
      driveLeftA.set(0);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(0);
      driveRightB.follow(driveRightA);
    }
}
