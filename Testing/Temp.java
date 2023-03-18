package frc.robot;

public class Temp {
    public static Thread armThread;
    public static boolean armRun;
    public static double targetArm;

    public static Thread extendThread;
    public static boolean extendRun;
    public static double targetExtend;

    public static void armStuff(double targetAngle, double targetExtension) {
        startArm(targetAngle);
        startExtend(targetExtension);
    }

    public static void startArm(double targetAngle) {
        armRun = true;
        targetArm = targetAngle;

        armThread = (() -> {
            while(Math.abs(armPotentiometer.get() - targetArm) > 0.01 && armRun) {
                //While the arm should run and isn't within a certain range of the target
                //It sets the armActuator speed to a proportion of the current angle - the target angle
                if(armPotentiometer.get() - targetArm < 0) {
                    //Use max so it doesn't try to set a speed below -1
                    armActuator.set(Math.max(-1, (armPotentiometer.get() - targetArm) * 10));
                } else {
                    //Use min so it doesn't try to set a speed above 1
                    armActuator.set(Math.min(1, (armPotentiometer.get() - targetArm) * 10));
                }

                try {
                    Thread.sleep(75);
                } catch(InterruptedException e) {

                }
            }
        });
        armThread.setPriority(Thread.MIN_PRIORITY);
        armThread.setDaemon(true);
        armThread.start();
    }

    public static void startExtend(double targetExtension) {
        extendRun = true;
        //Sorry about the confusing variable names
        targetExtend = targetExtension;

        extendThread = (() -> {
            while(Math.abs(armExtensionPotentiometer.get() - targetExtend) > 0.01 && extendRun) {
                //While the arm should run and isn't within a certain range of the target
                //It sets the armExtension speed to a proportion of the current extension - the target extension
                if(armExtensionPotentiometer.get() - targetExtend < 0) {
                    //Use max so it doesn't try to set a speed below -1
                    armExtension.set(Math.max(-1, (armExtensionPotentiometer.get() - targetExtend) * 10));
                } else {
                    //Use min so it doesn't try to set a speed above 1
                    armExtension.set(Math.min(1, (armExtensionPotentiometer.get() - targetExtend) * 10));
                }

                try {
                    Thread.sleep(75);
                } catch(InterruptedException e) {

                }
            }
        });
        extendThread.setPriority(Thread.MIN_PRIORITY);
        extendThread.setDaemon(true);
        extendThread.start();
    }

    public static void killArm() {
        armRun = false;
        targetArm = 0.0;
        //Thread should be automatically destroyed since it is no longer running?
        extendRun = false;
        targetExtend = 0.0;
    }
}
