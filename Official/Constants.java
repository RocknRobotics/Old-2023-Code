package frc.robot;
import java.util.*;
import java.io.*;

public class Constants
{
    public static final double RampUp = 2;

    public static boolean blueAlliance = true;

    public static final double balanceTolerance = 1.0;

    public static final double autoBackup = 0.0;

    public static final double armAngleTolerance = 0.0;
    public static final double armLengthTolerance = 0.0;

    public static final double topConeAngle = 0.465;
    public static final double topArmExtention = 0.645;

    public static final double GProportion = 9.80665;
    public static final double accelProportion = 1.0;
    //Position of game piece to pick up from station
    //index 0 == station 1, index 1 == 2, and index 2 == 3
    public static final double[] autoPieceY = new double[] {0.92, 2.14, 4.58};
    public static final double autoPieceX = 5.4;
    //Teeter midline, might be Z instead
    public static final double teeterPositionX = 2.51;

    //The amount of acceleration turning the motors to 1 gives
    public static final double accelConst = 1.0;
    //The maximum velocity the robot can travel in ft/s
    public static final double maxVelocity = 1.0;
    //Radius of the robot for the angle alignment (feet)
    public static final double robotRadius = 2.0;

    //Out-of-Bounds---Stores the X ranges and Z ranges of areas we don't want the robot to go to.
    //Each row stores two ranges---The lowest at index 0, and the highest at index 1
    public static final double[][] OBX = new double[][] {{}};
    public static final double[][] OBY = new double[][] {{}};

    //Under ideal cirumstances these are arrays relating to the OB arrays, but this should suffice for this year's game
    //Clearance around an obstacle
    public static final double xFeetClearance = 1.0;
    //Clearance before an obstacle
    public static final double yFeetClearance = 1.0;

    //The amount the position can be off by at any point of alignment on the curve
    public static final double xCurveTolerance = 0.1;
    public static final double yCurveTolerance = 0.1;

    //The amount the position can be off of the target position
    public static final double yPositionTolerance = 0.05;
    public static final double xPositionTolerance = 0.05;
    public static final double positionTolerance = 0.05;

    public static final double angleTolerance = 0.0;

    public static final double yLineTolerance = 0.0;

    //0 is top right---goes counter clockwise
    //Charge station is 247 cm wide (y) and 193 cm deep (x)
    //Center is 251 from far edge of grid tape line and centered in the width of the community
    //Grid tape is deep
    //Middle of community is 549 cm
    public static final double yTeeterClearance = 1.0;

    public static final double xTeeterClearance = 1.0;
    
    public static final double[] teeterCornersX = new double[] {2.51 + (1.93 / 2.0) + xTeeterClearance, 2.51 + (1.93 / 2.0) + xTeeterClearance, 2.51 - (1.93 / 2.0) - xTeeterClearance, 2.51 - (1.93 / 2.0) - xTeeterClearance};

    public static final double[] teeterCornersY = new double[] {5.49 - (2.47 / 2.0) + yTeeterClearance, 5.49 + (2.47 / 2.0) - yTeeterClearance, 5.49 + (2.47 / 2.0) - yTeeterClearance, 5.49 - (2.47 / 2.0) + yTeeterClearance};

    public static final double autoTeeterX = 0;

    public static final double autoTeeterY = 0;
    /*
    public static final String recordLoc = "";
    public static FileWriter recorder()
    {
        try{
            return new FileWriter(recordLoc, false);
        } catch (IOException e){}
        return null;
        
    }
    */

}
