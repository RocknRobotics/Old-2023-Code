
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
  //Parameter order
  //SPI.Port---The port used to connect to the navX (can be a I2C.Port instead) (this might just be a number, I'm not sure)
  //int---the bitrate of the sensor (max 2,000,000)
  //int---the update rate of the sensor sending us data (4 - 200)
  AHRS accelerometer = new AHRS(0, 0, 0);
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

  double accelTime = Timer.getFPGATimestamp();

  Thread accelThread;

  /*Accelerometer accelerometer = new BuiltInAccelerometer();
  double prevVelocityX = 0.0;
  double velocityX = 0.0;
  //Need to modify this based on starting station
  double prevPositionX = 0.0;
  double positionX = 0.0;
  double prevVelocityZ = 0.0;
  double velocityZ = 0.0;
  //Same here
  double prevPosition = 0.0;
  double positionZ = 0.0;
  //Having the arm facing fowards and perpendicular to the grid is considered 0.0
  //The left and right mototr velocities
  double leftVelocity = 0.0;
  double rightVelocity = 0.0;
  double angularVelocity = 0.0;
  double angle = 0.0;
  //You never know what might come in handy
  double velocity = 0.0;
  double position = 0.0;
  double accelTime = Timer.getFPGATimestamp();

  Thread accelThread;*/

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(0);
  AnalogPotentiometer armExtensionPotentiometer = new AnalogPotentiometer(1);

  //Arm threads
  Thread scoreConeTop;

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
    SmartDashboard.putNumber("Velocity X (left/right)", velocityX);
    SmartDashboard.putNumber("Velocity Z (Forwards/Backwards)", velocityZ);
    SmartDashboard.putNumber("Position X (left/right)", positionX);
    SmartDashboard.putNumber("Position Z (Forwards/Backwards)", positionZ);
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
        int numReceived = 0;

        for(double tempTime = Timer.getFPGATimestamp(); Timer.getFPGATimestamp() - tempTime < Math.pow(10, -5); tempTime = tempTime) {
          accelX += accelerometer.getWorldLinearAccelX() * Constants.GProportion;
          accelY += accelerometer.getWorldLinearAccelY() * Constants.GProportion;
          accelZ += accelerometer.getWorldLinearAccelZ() * Constants.GProportion;
          numReceieved++;
        }

        accelX /= (double) numReceived;
        accelY /= (double) numReceived;
        accelZ /= (double) numReceived;
        velocityX += (Timer.getFPGATimestamp() - accelTime) * accelX;
        velocityY += (Timer.getFPGATimestamp() - accelTime) * accelY;
        velocityZ += (Timer.getFPGATimestamp() - accelTime) * accelZ;
        positionX += (Timer.getFPGATimestamp() - accelTime) * velocityX;
        positionY += (Timer.getFPGATimestamp() - accelTime) * velocityY; 
        positionZ += (Timer.getFPGATimestamp() - accelTime) * velocityZ;

        double prevAngle = angle;
        double prevVelocity = angularVelocity;
        angle = accelerometer.getYaw();
        angularVelocity += (angle - prevAngle) / (Timer.getFPGATimestamp() - accelTime);
        angularAccel += (prevVelocity - angularVelocity) / (Timer.getFPGATimestamp() - accelTime);
      }
      /*//Right Riemann, if this is too innaccurate then create another set of variables to store previous velocity/position
      //and do the Middle (or do a trapezoidal if you're feeling fancy)
      while (1 + 6 == 7) {
        double prevZ = positionZ;
        velocityX += (Timer.getFPGATimestamp() - accelTime) * accelerometer.getX();
        positionX += (Timer.getFPGATimestamp() - accelTime) * velocityX;
        velocityZ += (Timer.getFPGATimestamp() - accelTime) * accelerometer.getZ();
        positionZ += (Timer.getFPGATimestamp() - accelTime) * velocityZ;
        accelTime = Timer.getFPGATimestamp();

        //What does get return? Need to figure this out
        leftVelocity += driveLeftA.get() * Constants.accelProportion * ((Timer.getFPGATimestamp() - accelTime));
        rightVelocity += driveRightA.get() * Constants.accelProportion * ((Timer.getFPGATimestamp() - accelTime));
        angularVelocity = (leftVelocity - rightVelocity) / Constants.robotRadius;
        angle += angularVelocity * (Timer.getFPGATimestamp() - accelTime);

        velocity = Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityZ, 2));
        position = Math.sqrt(Math.pow(positionX, 2) + Math.pow(positionZ, 2));
      }
      //try {
        //Thread.sleep(1);
      //} catch(InterruptedException e) {

      //}*/
    });
    //Low priority thread; minor increases in time between running shouldn't affect it too much
    accelThread.setPriority(Thread.MIN_PRIORITY);
    accelThread.start();

    scoreConeTop = new Thread(() -> {
      
    });
  }

  @Override
  public void autonomousInit() {
    accelerometer.zeroYaw();
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    if(!SmartDashboard.getBoolean("Go For Auto", true)) {
      return;
    }

    int station = -1;
    //Maybe it's Z, one of them won't change
    positionX = 0.0;

    if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 1;
      positionZ = 0.0;
    } else if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 2;
      positionZ = 0.0;
    } else if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 3;
      positionZ = 0.0;
    }

    if(SmartDashboard.getBoolean("Has Cone", true)) {
      //I'm not entirely sure but I'm hoping this will reset the variable value
      SmartDashboard.putBoolean("Has Cone", false);
      scoreCone();
    } else if(SmartDashboard.getBoolean("Has Cube", true)) {
      //I might change this to listen to the controller to know when it has a cube/cone and stuff, it depends
      SmartDashboard.putBoolean("Has Cube", false);
      scoreCube();
    }

    //This and other instances of -1 drive power might have to be changed to 1
    goTo(Constants.autoPieceX[station - 1], Constants.autoPieceZ[station - 1]);
      
    if(SmartDashboard.getBoolean("Grab Cone", true)) {
      SmartDashboard.putBoolean("Grab Cone", false);
      grabCone();
    } else if(SmartDashboard.getBoolean("Grab Cube", true)) {
      SmartDashboard.putBoolean("Grab Cube", false);
      grabCube();
    }

    goTo(Constants.autoTeeterX, Constants.autoTeeterZ);

    orient(0);
       
    //Points for docking are give 3 seconds after auto ends
    while(Timer.getFPGATimestamp() - autoStart <= 19) {
      goTo(Constants.teeterPositionX, positionZ);
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

      if (ps2.getOptionsButtonPressed()) {
        scoreConeTop();
      }

      if (ps2.getShareButtonPressed()) {
        balanceOnTeeter(100000, false, true, false);
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
    SmartDashboard.putNumber("Velocity X (left/right)", velocityX);
    SmartDashboard.putNumber("Velocity Z (Forwards/Backwards)", velocityZ);
    SmartDashboard.putNumber("Position X (left/right)", positionX);
    SmartDashboard.putNumber("Position Z (Forwards/Backwards)", positionZ);
    SmartDashboard.putNumber("Angular Velocity", angularVelocity);
    SmartDashboard.putNumber("Angle", angle);
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
    //Code for scoring a cone
    while (armPotentiometer.get() < 0.424) {
      armActuator.set(-1);
      if (armExtensionPotentiometer.get() < 0.645) {
        armExtension.set(-1);
      }
    }
    while (armPotentiometer.get() > 0.424) {
      armActuator.set(1);
      if (armExtensionPotentiometer.get() < 0.645) {
        armExtension.set(-1);
      }
    }
    while (armExtensionPotentiometer.get() < 0.64) {
      armExtension.set(-1);
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
  
  //PRECONDITION: The robot IS NOT moving and is in front of the teeter totter
  public void balanceOnTeeter(double time, boolean left, boolean middle, boolean right) {
    double z1 = Constants.teeterCornersZ[1];
    double x1 = Constants.teeterCornersX[1];
    double z2 = Constants.teeterCornersZ[2];
    double x2 = Constants.teeterCornersX[2];
    double z3 = Constants.teeterCornersZ[3];
    double x3 = Constants.teeterCornersX[3];
    double z0 = Constants.teeterCornersZ[0];
    double x0 = Constants.teeterCornersX[0];
    
    if(left) {
      if(Math.sqrt(Math.pow(z1 - positionZ, 2) + Math.pow(x1 - positionX, 2)) < 
      Math.sqrt(Math.pow(z2 - positionZ, 2) + Math.pow(x2 - positionX, 2))) {
        goTo(z1, x1, 0.0);
      } else {
        goTo(z2, x2, 180.0);
      }

    } else if(middle) {
      if(x0 - positionX < x3 - positionX) {
        goTo(x0, ((z1 - z0) / 2.0) + z0, 0.0);
      } else {
        goTo(x3, ((z1 - z0) / 2.0) + z0, 0.0);
      }

    } else if(right) {
      if(Math.sqrt(Math.pow(z3 - positionZ, 2) + Math.pow(x3 - positionX, 2)) < 
      Math.sqrt(Math.pow(z0 - positionZ, 2) + Math.pow(x0 - positionX, 2))) {
        goTo(z3, x3, 0.0);
      } else {
        goTo(z0, x0, 180.0);
      }

    }

    while(Timer.getFPGATimestamp() - autoStart < time) {
      goTo(Constants.teeterPositionX, positionZ, angle);
    }
  }

  public void goTo(double newPositionX, double newPositionZ, double newAngle) {
    if(isOB(newPositionX, newPositionZ)) {
      return;
    }
    double slope = (newPositionX - positionX) / (newPositionZ - positionZ);
    double tempX = positionX;
    double tempZ = positionZ;
    ArrayList<Double> curvePointsZ = new ArrayList<Double>();
    ArrayList<Double> curvePointsX = new ArrayList<Double>();
    for(int i = 0; i < Constants.OBZ.length; i++) {
      if(tempZ < Constants.OBZ[i][0] && newPositionZ > Constants.OBZ[i][1]) {
        double startX = (Constants.OBZ[i][0] - tempZ) * slope + tempX;
        double endX = (Constants.OBZ[i][1] - tempZ) * slope + tempX;
        if((startX >= Constants.OBX[i][0] && startX <= Constants.OBX[i][1]) || (endX >= Constants.OBX[i][0] && endX <= Constants.OBX[i][1])) {
          curvePointsZ.add(tempZ);
          curvePointsX.add(tempX);
          if(newPositionZ < positionZ) {
            curvePointsZ.add(Constants.OBZ[i][1]);
            tempZ = Constants.OBZ[i][1];
            if(Constants.OBZ[i][0] > newPositionZ) {
              curvePointsZ.add(Constants.OBZ[i][0]);
              tempZ = Constants.OBZ[i][0];
            }
          } else {
            curvePointsZ.add(Constants.OBZ[i][0]);
            tempZ = Constants.OBZ[i][0];
            if(Constants.OBZ[i][1] < newPositionZ) {
              curvePointsZ.add(Constants.OBZ[i][1]);
              tempZ = Constants.OBZ[i][1];
            }
          }
          if(isOB(Constants.OBZ[i][0], Constants.OBX[i][0] - 1.0)) {
            curvePointsX.add(Constants.OBX[i][1]);
            tempX = Constants.OBX[i][1];
          } else if(isOB(Constants.OBZ[i][0], Constants.OBX[i][1] - 1.0)) {
            curvePointsX.add(Constants.OBX[i][0]);
            tempX = Constants.OBX[i][0];
          } else if(Math.abs(startX - Constants.OBX[i][0]) < Math.abs(startX - Constants.OBX[i][1])) {
            curvePointsX.add(Constants.OBX[i][0]);
            tempX = Constants.OBX[i][0];
          } else {
            curvePointsX.add(Constants.OBX[i][1]);
            tempX = Constants.OBX[i][1];
          }
          
          if(curvePointsX.size() < curvePointsZ.size()) {
            curvePointsX.add(curvePointsX.get(curvePointsX.size() - 1));
          }
        }
      }
    }
    curvePointsZ.add(newPositionZ);
    curvePointsX.add(newPositionX);
    while(curvePointsZ.size() > 0) {
      double currTime = Timer.getFPGATimestamp();
      double targetZ = curvePointsZ.remove(0);
      double targetX = curvePointsX.remove(0);
      double zLine = positionZ;
      while(Math.abs(targetX - positionX) > Constants.xPositionTolerance && Math.abs(targetZ - positionZ) > Constants.zPositionTolerance) {
        while((Math.abs(targetX - positionX) <= Constants.xPositionTolerance && Math.abs(targetZ - positionZ) > Constants.zPositionTolerance)
        || Math.abs(zLine - positionZ) > Constants.zLineTolerance) {
          if(targetX - positionX <= Constants.xPositionTolerance) {
            if(Math.abs((positionZ < targetZ ? 90.0: 270.0) - angle) >= Constants.angleTolerance) {
              orient(positionZ < targetZ ? 90.0: 270.0);
            }
          } else {
            if(Math.abs((positionZ < zLine ? 90.0 : 270.0) - angle) >= Constants.angleTolerance) {
              orient(positionZ < zLine ? 90.0 : 270.0);
            }
          }
          driveLeftA.set(1);
          driveRightA.set(1);
        }
        if(Math.abs((targetX > positionX ? 180.0 : 0.0) - angle) > Constants.angleTolerance) {
          orient(targetX > positionX ? 180.0 : 0.0);
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
      /**while((Timer.getFPGATimestamp() - tempTime) < Math.pow(10, -10)) {}
      double prevAngularVelocity = 0.0;
      double angularVelocity = (angle - tempAngle) / (Timer.getFPGATimestamp() - tempTime);
      double angularAccel = (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - tempTime);
      tempAngle = angle;
      tempTime = Timer.getFPGATimestamp();*/
      //Might have to change to quarter-circle calculations instead
      while(Math.abs(targetAngle - (angle + (((angularVelocity / angularAccel) * angularVelocity) / 2.0))) > Constants.angleTolerance) {
        /**prevAngularVelocity = angularVelocity;
        angularVelocity += (angle - tempAngle) / (Timer.getFPGATimestamp() - tempTime);
        angularAccel += (angularVelocity - prevAngularVelocity) / (Timer.getFPGATimestamp() - tempTime);
        tempAngle = angle;
        tempTime = Timer.getFPGATimestamp();*/
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

  public boolean isOB(double aPositionX, double aPositionZ) {
    for(int i = 0; i < Constants.OBX.length; i++) {
      if(aPositionX >= Constants.OBX[i][0] && aPositionX <= Constants.OBX[i][1] && aPositionZ >= Constants.OBZ[i][0] && aPositionZ <= Constants.OBZ[i][1]) {
        return true;
      }
    }

    return false;
  }
}
