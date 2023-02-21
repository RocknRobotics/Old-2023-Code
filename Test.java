
/*
  This is catastrophically poorly written code for the sake of being easy to follow
  If you know what the word "refactor" means, you should refactor this code
*/
package frc.robot;

import java.io.OutputStream;

import java.text.*;

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
  DoubleSolenoid clawSolenoid1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid clawSolenoid2 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 1, 0);
  

  // accelerometer
  Accelerometer accelerometer = new BuiltInAccelerometer();
  double velocityX = 0.0;
  //Need to modify this based on starting station
  double positionX = 0.0;
  double velocityZ = 0.0;
  //Same here
  double positionZ = 0.0;
  //Having the arm facing fowards and perpendicular to the grid is considered 0.0
  double angle = 0.0;
  //You never know what might come in handy
  double velocity = 0.0;
  double position = 0.0;
  double accelTime = Timer.getFPGATimestamp();

  Thread accelThread;

  //Potentiometer
  AnalogPotentiometer armPotentiometer = new AnalogPotentiometer(0);

  //Controller
  PS4Controller ps1 = new PS4Controller(0);

  //Camera
  Thread m_visionThread;

  double prev = 0;
  double autoStart = 0;
  boolean goForAuto = false;
  boolean fast = false;

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
    
    driveLeftA.setOpenLoopRampRate(200);
    driveLeftB.setOpenLoopRampRate(200);
    driveRightA.setOpenLoopRampRate(200);
    driveRightB.setOpenLoopRampRate(200);
    driveLeftA.burnFlash();
    driveLeftB.burnFlash();
    driveRightA.burnFlash();
    driveRightB.burnFlash();
    
    fast = false;

    //Pneumatics
    PneumaticsControl.enableCompressorAnalog(100, 120);
    PneumaticsCompressor.enableAnalog(100, 120);

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
    SmartDashboard.putBoolean("Station 1", false);
    //Middle
    SmartDashboard.putBoolean("Station 2", false);
    //Right far end
    SmartDashboard.putBoolean("Station 3", false);
    //Cone or Cube to pick up
    SmartDashboard.putBoolean("Cone", false);
    SmartDashboard.putBoolean("Cube", false);

    accelThread = () -> {
      //Right Riemann, if this is too innaccurate then create another set of variables to store previous velocity/position
      //and do the Middle (or do a trapezoidal if you're feeling fancy)
      velocityX += (Timer.getFPGATimestamp() - accelTime) * accelerometer.getX();
      positionX += (Timer.getFPGATimestamp() - accelTime) * velocityX;
      velocityZ += (Timer.getFPGATimestamp() - accelTime) * accelerometer.getZ();
      positionZ += (Timer.getFPGATimestamp() - accelTime) * velocityZ;
      accelTime = Timer.getFPGATimestamp();

      if(velocityX == 0) {
        if(velocityZ == 0) {
          //Staying still
          angle = angle;
        } else if(velocityZ > 0) {
          //Heading to the right
          angle = 90.0;
        } else {
          //Heading to the left
          angle = 270.0;
        }
      } else if(velocityX > 0) {
        if(velocityZ == 0) {
          //Heading "up"
          angle = 180.0;
        } else if(velocityZ > 0) {
          //Heading "up" and right
          angle = Math.atan(velocityX / velocityZ) + 90.0;
        } else {
          //Heading "up" and left
          angle = Math.atan(velocityX / velocityZ) + 180.0;
        }
      } else {
        if(velocityZ == 0) {
          //Heading "down"
          angle = 0.0;
        } else if(velocityZ > 0) {
          //Heading "down" and right
          angle = Math.atan(velocityX / velocityZ);
        } else {
          //Heading "down" and left
          angle = Math.atan(velocityX / velocityZ) + 270.0;
        }
      }

      velocity = Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityZ, 2));
      position = Math.sqrt(Math.pow(positionX, 2) + Math.pow(positionZ, 2));
    };
    //Low priority thread; minor increases in time between running shouldn't affect it too much
    accelThread.setDaemon(true);
    accelThread.start();
  }

  @Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto
    goForAuto = SmartDashboard.getBoolean("Go For Auto", true);
    
    if(SmartDashboard.getBoolean("Station 1", true)) {
      positionX = 0.0;
      positionZ = 0.0;

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
      positionX = 0.0;
      positionZ = 0.0;
      
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
      positionX = 0.0;
      positionZ = 0.0;
      
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
    boolean stopped = false;
    while (stopped) {
      if (ps1.getPSButtonPressed()) {
        stopped = false;
      }
    }

    if (ps1.getPSButtonPressed()) {
      stopped = true;
    }
    
    double num = armPotentiometer.get();
    num = (int)(num * 100) / 100.0;
    if (num != prev) {
      System.out.println((num) + " - ");
      prev = num;
    }
    

    // Set up arcade steer
    double forward = 0;
    double turn = 0;

    //System.out.println(driveLeftA.getClosedLoopRampRate());
    //System.out.println(driveLeftA.getOpenLoopRampRate());

    // xbox
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
    double driveLeftPower = (forward + turn);
    double driveRightPower = (forward - turn);

    driveLeftA.set(driveLeftPower / 2);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(driveRightPower / 2);
    driveRightB.follow(driveRightA);

    //arm testing
    if (ps1.getCircleButton()) {
      armActuator.set(1);
    } else if (ps1.getCrossButton()) {
      armActuator.set(-1);
    } else {
      armActuator.set(0);
    }

    if (ps1.getL1Button()) {
      armExtension.set(1);
    } else if (ps1.getR1Button()) {
      armExtension.set(-1);
    } else {
      armExtension.set(0);
    }

    if (ps1.getOptionsButton()) {
      clawSolenoid1.set(DoubleSolenoid.Value.kForward);
      clawSolenoid2.set(DoubleSolenoid.Value.kForward);
    } else if (ps1.getShareButton()) {
      clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
      clawSolenoid2.set(DoubleSolenoid.Value.kReverse);
    } 

    if (ps1.getR3Button()) {
      PneumaticsCompressor.start();
    } else if (ps1.getL3Button()) {
      PneumaticsCompressor.stop();
    }

    //Drivers MUST have the robot completely stopped sometime during the last 15 seconds for the robot to auto teeter. They also must stop it in front of the docking station
    if(Timer.getFPGATimestamp() - autoStart >= 135 && accelerometer.getX() == 0 && accelerometer.getZ() == 0) {
      teeter(150.0);
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
    armActuator.set(0);
    armExtension.set(0);
    PneumaticsControl.disableCompressor();
    clawSolenoid1.set(DoubleSolenoid.Value.kOff);
    clawSolenoid2.set(DoubleSolenoid.Value.kOff);
    // intake.set(ControlMode.PercentOutput, 0);
  }

  public void scoreCone() {
    //Code for scoring a cone
  }
  public void scoreCube() {
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

  public void goTo(double newPositionX, double newPositionZ) {
    if(isOB(newPositionX, newPositionZ)) {
      return;
    }

    double slope = (newPositionX - positionX) / (newPositionZ - positionZ);
    double tempX = positionX;
    double tempZ = positionZ;
    double newPosition = Math.sqrt(Math.pow(newPositionX, 2) + Math.pow(newPositionZ, 2));
    double targetAngle;

    //Index 0 is when it should start "curving", Index 1 is the vertex, Index 2 is the end point
    ArrayList<double[]> startCurvesX = new ArrayList<double[]>();
    ArrayList<double[]> startCurvesZ = new ArrayList<double[]>();

    while(tempZ <= newPositionZ) {
      tempZ += 0.1;
      tempX += slope * 0.1;

      if(isOB(tempX, tempZ)) {
        //Treat the Z as being fine
        double[] xCurve = new double[3];
        double[] zCurve = new double[3];
        double needToGoDown = tempX;
        double needToGoUp = tempX;

        while(isOB(needToGoDown, tempZ) && isOB(needToGoUp, tempZ)) {
          needToGoDown -= 0.1;
          needToGoUp += 0.1;
        }

        if(isOB(needToGoDown, tempZ)) {
          //Go up
          needToGoUp += xMeterClearance;
          double zHeight = Math.tan(xAngleClearance) * needToGoUp;
          zCurve[0] = tempZ - (zHeight + zMeterClearance);
          zCurve[1] = tempZ;
          zCurve[2] = tempZ + zHeight + zMeterClearance;

          xCurve[0] = tempX;
          xCurve[1] = needToGoUp;
          xCurve[2] = tempX;

          startCurvesX.add(xCurve);
          startCurvesZ.add(zCurve);
        } else {
          //Go down
          needToGoDown -= xMeterClearance;
          double zHeight = Math.tan(xAngleClearance) * needToGoDown * -1;
          zCurve[0] = tempZ - (zHeight + zMeterClearance);
          zCurve[1] = tempZ;
          zCurve[2] = tempZ + zHeight + zMeterClearance;

          xCurve[0] = tempX;
          xCurve[1] = needToGoDown;
          xCurve[2] = tempX;

          startCurvesX.add(xCurve);
          startCurvesZ.add(zCurve);
        }
      }
    }

    targetAngle = calcAngle(newPositionX, positionX, newPositionZ, positionZ);

    orient(targetAngle);

    driveLeftA.set(-1);
    driveLeftB.follow(driveLeftA);
    driveRightA.follow(driveLeftA);
    driveRightB.follow(driveLeftA);

    while(Math.abs(newPositionZ - positionZ) > zPositionTolerance && Math.abs(newPositionX - positionX) > xPositionTolerance) {
      if(startCurvesZ.size() > 0 && startCurvesX.size() > 0) {
        double[] zCurve = startCurvesZ.get(0);
        double[] xCurve = startCurvesX.get(0);
  
        if(Math.abs(zCurve[0] - positionZ) <= zCurveTolerance && Math.abs(xCurve[0] - positionZ <= xCurveTolerance)) {
          targetAngle = calcAngle(xCurve[1], positionX, zCurve[1], positionZ);

          orient(targetAngle);
          
          driveLeftA.set(-1);
          driveLeftB.follow(driveLeftA);
          driveRightA.follow(driveLeftA);
          driveRightB.follow(driveLeftA);
  
          while(Math.abs(zCurve[1] - positionZ) > zCurveTolerance && Math.abs(xCurve[1] - positionX) > xCurveTolerance) {}
          targetAngle = calcAngle(xCurve[2], positionX, zCurve[2], positionZ);
  
          orient(targetAngle);
  
          driveLeftA.set(-1);
          driveLeftB.follow(driveLeftA);
          driveRightA.follow(driveLeftA);
          driveRightB.follow(driveLeftA);
  
          while(Math.abs(zCurve[2] - positionZ) > zCurveTolerance && Math.abs(xCurve[2] - positionX) > xCurveTolerance) {}
          targetAngle = calcAngle(newPositionX, positionX, newPositionZ, positionZ);
          
          orient(targetAngle);
          startCurvesZ.remove(0);
          startCurvesX.remove(0);
          
          driveLeftA.set(-1);
          driveLeftB.follow(driveLeftA);
          driveRightA.follow(driveLeftA);
          driveRightB.follow(driveLeftA);
        } else {
          driveLeftA.set(-1);
          driveLeftB.follow(driveLeftA);
          driveRightA.follow(driveLeftA);
          driveRightB.follow(driveLeftA);
          double tempTime = Timer.getFPGATimestamp();
  
          //Might need changing
          while(Timer.getFPGATimestamp() - tempTime <= Math.pow(10, -5)) {}
        }
      } else if(Math.abs(newPostition - (velocity / accelConst * velocity / 2 + position)) <= positionTolerance) {
        //Changes direction of acceleration if it will cause it to stop upon reaching its destination
        driveLeftA.set(1);
        driveLeftB.follow(driveLeftA);
        driveRightA.follow(driveLeftA);
        driveRightB.follow(driveLeftA);
      }
    }
  }

  public boolean isOB(double aPositionX, double aPositionZ) {
    for(int i = 0; i < OBX.length; i++) {
      if(aPositionX >= OBX[i][0] && aPositionX <= OBX[i][1] && aPositionZ >= OBZ[i][0] && aPostionZ <= OBZ[i][1]) {
        return true;
      }
    }

    return false;
  }

  public double calcAngle(double newPositionX, double oldPositionX, double newPositionZ, double oldPositionZ) {
    if(newPositionX == oldPositionX) {
      if(newPositionZ == oldPositionZ) {
        //Staying still
        return angle;
      } else if(newPositionZ > oldPositionZ) {
        //Heading to the right
        return 90.0;
      } else {
        //Heading to the left
        return 270.0;
      }
    } else if(newPositionX > oldPositionX) {
      if(newPositionZ == oldPositionZ) {
        //Heading "up"
        return 180.0;
      } else if(newPositionZ > oldPositionZ) {
        //Heading "up" and right
        return Math.atan((newPositionX - oldPositionX) / (newPositionZ - oldPositionZ)) + 90.0;
      } else {
        //Heading "up" and left
        return Math.atan((newPositionX - oldPositionX) / (newPositionZ - oldPositionZ)) + 180.0;
      }
    } else {
      if(newPositionZ == oldPositionZ) {
        //Heading "down"
        return 0.0;
      } else if(newPositionZ > oldPositionZ) {
        //Heading "down" and right
        return Math.atan((newPositionX - oldPositionX) / (newPositionZ - oldPositionZ));
      } else {
        //Heading "down" and left
        return Math.atan((newPositionX - oldPositionX) / (newPositionZ - oldPositionZ)) + 270.0;
      }
    }

    return 0.0;
  }

  public void orient(double targetAngle) {
    //I promise it'll be done once I have access to the equation wall
  }
}
