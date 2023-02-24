
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

  //Pneumatics
  PneumaticsControlModule PneumaticsControl = new PneumaticsControlModule();
  Compressor PneumaticsCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid clawSolenoid1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid clawSolenoid2 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
  

  // accelerometer
  Accelerometer accelerometer = new BuiltInAccelerometer();

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(0);
  AnalogPotentiometer armExtensionPotentiometer = new AnalogPotentiometer(0);

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


  //Finals
  //Used for teeter totter balancing
  final double accelProportion = 1.0;
  //Time that it takes the robot to drive back to a new piece
  final double autoBackUpTime = 0.0;
  //Time that it takes the robot to drive back to a new piece from Station 2
  final double autoBackUpTime2 = 0.0;
  //Time for the robot to turn around so it can grab a piece---Should be the same for all stations
  final double turnTime =  0.0;
  //Time for robot to orient towards the teeter totter
  final double teeterTurnTime = 0.0;
  //Time for the robot to drive backwards towards the teeter totter
  final double teeterDriveTime = 0.0;
  //Station 2 code
  final double teeterDriveTime2 = 0.0;
  //Time stamp for the robot to line up to not take up all the space on the teeter totter
  final double teeterOrientTime = 0.0;
  //Time to stay on the teeter
  final double onTeeterTime = 0.0;
  //Postion that the robot must be at on the teeter totter for it to be stable, assuming it starts just in front of the teeter totter.
  final double teeterPosition = 0.0;


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
    SmartDashboard.putNumber("accelerometer X", 0);
    SmartDashboard.putNumber("accelerometer Z", 0);
    SmartDashboard.putNumber("accelerometer Y", 0);
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());

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
      driveRightB.follow(driveLeftA);

      while(Timer.getFPGATimestamp() - autoStart <= autoBackUpTime) {}

      driveLeftA.set(0);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1);
      driveLeftB.follow(driveRightA);

      while(Timer.getFPGATimestamp() - autoStart <= turnTime + autoBackUpTime) {}
      
      if(SmartDashboard.getBoolean("Cone", true)) {
        grabCone();
      } else if(SmartDashboard.getBoolean("Cube", true)) {
        grabCube();
      }

      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(0);
      driveRightB.follow(driveRightA);

      while(Timer.getFPGATimestamp() - autoStart <= teeterTurnTime + turnTime + autoBackUpTime) {}
       
      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveLeftA);

      while(Timer.getFPGATimestamp() - autoStart <= teeterDriveTime + teeterTurnTime + turnTime + autoBackUpTime) {}

      driveLeftA.set(0);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1);
      driveRightB.follow(driveRightA);

       while(Timer.getFPGATimestamp() - autoStart <= teeterOrientTime + teeterDriveTime + teeterTurnTime + turnTime + autoBackUpTime) {}
       
    } else if(SmartDashboard.getBoolean("Station 2", true)) {
      scoreCone();

      //Something something maybe 1 instead of -1 I dunno
      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveLeftA);

      while(Timer.getFPGATimestamp() - autoStart <= autoBackUpTime2) {}

      driveLeftA.set(0);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(-1);
      driveRightB.follow(driveRightA);

      if(SmartDashboard.getBoolean("Cone", true)) {
        grabCone();
      } else if(SmartDashboard.getBoolean("Cube", true)) {
        grabCube();
      }

      while(Timer.getFPGATimestamp() - autoStart <= turnTime + autoBackUpTime2) {}

      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveLeftA);

      while(Timer.getFPGATimestamp() - autoStart <= teeterDriveTime2 + turnTime + autoBackUpTime2) {}

    } else if(SmartDashboard.getBoolean("Station 3", true)) {
      scoreCone();

      //This and other instances of -1 drive power might have to be changed to 1
      driveRightA.set(-1);
      driveRightB.follow(driveRightA);
      driveLeftA.follow(driveRightA);
      driveLeftB.follow(driveRightA);

      while(Timer.getFPGATimestamp() - autoStart <= autoBackUpTime) {}

      driveRightA.set(0);
      driveRightB.follow(driveRightA);
      driveLeftA.set(-1);
      driveLeftB.follow(driveLeftA);

      while(Timer.getFPGATimestamp() - autoStart <= turnTime + autoBackUpTime) {}
      
      if(SmartDashboard.getBoolean("Cone", true)) {
        grabCone();
      } else if(SmartDashboard.getBoolean("Cube", true)) {
        grabCube();
      }

       driveRightA.set(-1);
       driveRightB.follow(driveRightA);
       driveLeftA.set(0);
       driveLeftB.follow(driveLeftA);

       while(Timer.getFPGATimestamp() - autoStart <= teeterTurnTime + turnTime + autoBackUpTime) {}

       driveRightA.set(-1);
       driveRightB.follow(driveRightA);
       driveLeftA.follow(driveRightA);
       driveLeftB.follow(driveRightA);

       while(Timer.getFPGATimestamp() - autoStart <= teeterDriveTime + teeterTurnTime + turnTime + autoBackUpTime) {}

       driveRightA.set(0);
       driveRightB.follow(driveRightA);
       driveLeftA.set(-1);
       driveLeftB.follow(driveLeftA);

       while(Timer.getFPGATimestamp() - autoStart <= teeterOrientTime + teeterDriveTime + teeterTurnTime + turnTime + autoBackUpTime) {}
    }

    double tempTime = Timer.getFPGATimestamp();

    driveLeftA.set(-1);
    driveLeftB.follow(driveLeftA);
    driveRightA.follow(driveLeftA);
    driveRightB.follow(driveLeftA);

    while(Timer.getFPGATimestamp() - tempTime <= onTeeterTime) {}
    
    //Sets acceleration to 0
    driveLeftA.set(0);
    driveLeftB.follow(driveLeftA);
    driveRightA.follow(driveLeftA);
    driveRightB.follow(driveLeftA);
    
    //Might have to be getZ
    while(accelerometer.getX() != 0) {}

    balanceOnTeeter(15.0);
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
    if (!stopped2) {
      if (ps2.getL2Axis() > 0.5) {
        armActuator.set(1);
      } else if (ps2.getR2Axis() > 0.5) {
        armActuator.set(-1);
      } else {
        armActuator.set(0);
      }

      if (ps2.getLeftY() < -0.5) {
        armExtension.set(1);
      } else if (ps2.getLeftY() > 0.5) {
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
      } else if (ps2.getCrossButton() && !closed) {
        clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
        clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
        closed = true;
      } 

      if (ps2.getTriangleButtonPressed() && topcam == false) {
        cameraSelection.setString(topCamera.getName());
        topcam = true;
      } else if (ps2.getTriangleButtonPressed() && topcam == true) {
        cameraSelection.setString(bottomCamera.getName());
        topcam = false;
      }

      if (ps2.getR3Button()) {
        PneumaticsCompressor.start();
      } else if (ps2.getL3Button()) {
        PneumaticsCompressor.stop();
      }
    }

    //Drivers MUST have the robot completely stopped sometime during the last 15 seconds for the robot to auto teeter. They also must stop it in front of the docking station
    if(Timer.getFPGATimestamp() - autoStart >= 135 && accelerometer.getX() == 0 && accelerometer.getZ() == 0) {
      teeter(150.0);
    }

    //Reading measurements
    SmartDashboard.putNumber("accelerometer X", accelerometer.getX());
    SmartDashboard.putNumber("accelerometer Z", accelerometer.getZ());
    SmartDashboard.putNumber("accelerometer Y", accelerometer.getY());
    SmartDashboard.putNumber("arm potentiometer", armPotentiometer.get());
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
    //Code for scoring a cone
    while (armPotentiometer.get() < 0.424) {
      armActuator.set(1);
    }
    while (armPotentiometer.get() > 0.424) {
      armActuator.set(-1);
    }
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
  
  //PRECONDITION: The robot IS NOT moving and is in front of the teeter totter
  public void balanceOnTeeter(double time) {
    double currTime = Timer.getFPGATimestamp();
    double position = 0.0;
    double velocity = 0.0;
    
    while(Timer.getFPGATimestamp() - autoStart < time) {
      //Yet another place where it might have to be getZ instead of getX
      velocity += (Timer.getFPGATimestamp() - currTime) * accelerometer.getX();
      position += (Timer.getFPGATimestamp() - currTime) * velocity;
      currTime = Timer.getFPGATimestamp();
      
      if(position < teeterPosition) {
        //Maybe -1, who knows. Not me, that's for sure
        driveLeftA.set(1);
      } else if(position > teeterPosition) {
        driveLeftA.set(-1);
      }
      
      driveLeftB.follow(driveLeftA);
      driveRightA.follow(driveLeftA);
      driveRightB.follow(driveLeftA);
      
      //Might have to change the Math.pow to be more or less, we'll see
      while(Timer.getFPGATimestamp() - currTime < Math.pow(10, -5)) {}
    }
  }

  public void teeter(double a) {

  }
}
