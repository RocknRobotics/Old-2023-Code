import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class laptopMain {
    static Thread autoIntake;
    //Height == 288
    //Width == 352
    static HttpCamera bottomCam = new HttpCamera("Bottom", "http://roborio-3692-frc.local:1181/?action=stream");
    static CvSink bottomSink = CameraServer.getVideo(bottomCam);
    static Mat img = new Mat();

    static final int[] yellowLows = {0, 0, 0};
    static final int[] yellowHighs = {0, 0, 0};
    static final int[] cubeLows = {0, 0, 0};
    static final int[] cubeHighs = {0, 0, 0};

    public static void main(String[] args) {
        System.loadLibrary("ntcore");
        System.loadLibrary("ntcorejni");
        System.loadLibrary("opencv_java460");
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("team3692-frc2023");
        inst.setServerTeam(3692, NetworkTableInstance.kDefaultPort4);
        SmartDashboard.setNetworkTableInstance(inst);

        autoIntake = new Thread(() -> {
            while(!false) {
                bottomSink.grabFrame(img);

                System.out.println(img.type());

                try {
                    Thread.sleep(75);
                } catch(InterruptedException e) {

                }
            }
        });
        autoIntake.setPriority(Thread.MAX_PRIORITY);
        autoIntake.setDaemon(false);
        autoIntake.start();
    }
}
