public class Constants
{
    public static final double RampUp = 2;

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

    //The amount of acceleration turning the motors to 1 gives
    final double accelConst = 1.0;

    //Out-of-Bounds---Stores the X ranges and Z ranges of areas we don't want the robot to go to.
    //Each row stores two ranges---The lowest at index 0, and the highest at index 1
    final double[][] OBX = new double[][] {{}};
    final double[][] OBZ = new double[][] {{}};

    final double xAngleClearnace = 15.0;
    final double xMeterClearance = 1.0;
    final double zMeterClearance = 1.0;

    //The amount the position can be off by at any point of alignment on the curve
    final double xCurveTolerance = 0.1;
    final double zCurveTolerance = 0.1;

    //The amount the position can be off of the target position
    final double zPositionTolerance = 0.05;
    final double xPositionTolerance = 0.05;
    final double positionTolerance = 0.05;

    //Stores coordinates of teeter totter (since while it can be navigated, it requires correct positioning)
    final double[] teeterCorners = new double[] {};
}
