import com.fazecast.jSerialComm.*;

//Make sure raspberry pi has jar file and wpilib library

public class Raspberry {
    //This is just for recording ports, remove once all ports are matched up
    SerialPort[] thePorts = SerialPort.getCommPorts();

    //Gives description for each SP, mark them down and then trial and error
    for(SerialPort aPort: thePorts) {
        System.out.println(aPort.getPortDescription());
    }

    //Need to put correct port descriptor as parameter
    //Set update rate to 50 to start out
    SerialPort accelerometer1 = new SerialPort();
    SerialPort accelerometer2 = new SerialPort();
    Thread accel1;
    Thread accel2;
    boolean oneFinished;
    boolean twoFinished;
    //accelerometer1, accelerometer2, average
    double[] accelX = new double[3];
    double[] accelY = new double[3];
    double[] accelZ = new double[3];
    double[] velocityX = new double[3];
    double[] velocityY = new double[3];
    double[] velocityZ = new double[3];
    double[] positionX = new double[3];
    double[] positionY = new double[3];
    double[] positionZ = new double[3];
    double[] angularAccel = new double[3];
    double[] angularVelocity = new double[3];
    double[] angle = new double[3];



    public Raspberry() {
        accel1 = new Thread(() -> {
            //Need to use byte updating


            oneFinished = true;

            if(oneFinished && twoFinished) {
                oneFinished = false;
                twoFinished = false;

                accelX[2] = (accelX[0] + accelX[1]) / 2.0;
                accelY[2] = (accelY[0] + accelY[1]) / 2.0;
                accelZ[2] = (accelZ[0] + accelZ[1]) / 2.0;
                velocityX[2] = (velocityX[0] + velocityX[1]) / 2.0;
                velocityY[2] = (velocityY[0] + velocityY[1]) / 2.0;
                velocityZ[2] = (velocityZ[0] + velocityZ[1]) / 2.0;
                positionX[2] = (positionX[0] + positionX[1]) / 2.0;
                positionY[2] = (positionY[0] + positionY[1]) / 2.0;
                positionZ[2] = (positionZ[0] + positionZ[1]) / 2.0;
                angularAccel[2] = (angularAccel[0] + angularAccel[1]) / 2.0;
                angularVelocity[2] = (angularVelocity[0] + angularVelocity[1]) / 2.0;
                angle[2] = (angle[0] + angle[1]) / 2.0;

                //Update Network Tables
                
            }
        });
        accel1.setDaemon(true);
        accel1.start();

        accel2 = new Thread(() -> {
            //Need to use byte updating


            twoFinished = true;

            if(oneFinished && twoFinished) {
                oneFinished = false;
                twoFinished = false;

                accelX[2] = (accelX[0] + accelX[1]) / 2.0;
                accelY[2] = (accelY[0] + accelY[1]) / 2.0;
                accelZ[2] = (accelZ[0] + accelZ[1]) / 2.0;
                velocityX[2] = (velocityX[0] + velocityX[1]) / 2.0;
                velocityY[2] = (velocityY[0] + velocityY[1]) / 2.0;
                velocityZ[2] = (velocityZ[0] + velocityZ[1]) / 2.0;
                positionX[2] = (positionX[0] + positionX[1]) / 2.0;
                positionY[2] = (positionY[0] + positionY[1]) / 2.0;
                positionZ[2] = (positionZ[0] + positionZ[1]) / 2.0;
                angularAccel[2] = (angularAccel[0] + angularAccel[1]) / 2.0;
                angularVelocity[2] = (angularVelocity[0] + angularVelocity[1]) / 2.0;
                angle[2] = (angle[0] + angle[1]) / 2.0;

                //Update Network Tables
                
            }
        });
        accel2.setDaemon(true);
        accel2.start();
    }
}