package frc.robot.vision;


import edu.wpi.first.hal.NotifierJNI;
import edu.wpi.first.wpilibj.RobotController;

public class Pixy2USBOpenCVCalibrate {
    static {
       System.loadLibrary("pixy2_usb");
    }

    private Thread m_acquire_task;

    private class AcquireTask implements Runnable {
        private Pixy2USBOpenCVCalibrate pixy2usbOpenCVCalibrate;
        private double m_expirationTime;
        private int m_notifier = NotifierJNI.initializeNotifier();
        private final double m_period = 2.0;

        public AcquireTask(Pixy2USBOpenCVCalibrate pixy2usbOpenCVCalibrate) {
            this.pixy2usbOpenCVCalibrate = pixy2usbOpenCVCalibrate;
        }

        @Override
        public void run() {
            System.out.println("INFO: Starting Pixy2USBOpenCVCalibrate");

            int init_result = pixy2usbOpenCVCalibrate.pixy2USBInit();
            if (init_result == 0) {
                pixy2usbOpenCVCalibrate.pixy2USBGetVersion();
                pixy2usbOpenCVCalibrate.pixy2USBLampOff();

                m_expirationTime = RobotController.getFPGATime() * 1e-6 + m_period;
                updateAlarm();

                for(int i=0; i<1; ++i) {
                    long curTime = NotifierJNI.waitForNotifierAlarm(m_notifier);
                    if(curTime == 0) {
                        break;
                    }
    
                    m_expirationTime += m_period;
                    updateAlarm();
    
                    System.out.println("TODO: Should take picture here");
                    // pixy2usbOpenCVCalibrate.pixy2USBTakePicture();
                }

                pixy2usbOpenCVCalibrate.pixy2Calibrate();

            } else {
                System.out.println("WARNING: Failed Pixy2 USB Init!!!");
            }
        }

        private void updateAlarm(){
            NotifierJNI.updateNotifierAlarm(m_notifier, (long) (m_expirationTime * 1e6));
        }
    }

    public Pixy2USBOpenCVCalibrate() {
        m_acquire_task = new Thread(new AcquireTask(this));
        m_acquire_task.setDaemon(true);
        m_acquire_task.start();
    }

    private native int pixy2USBInit();

    private native void pixy2USBGetVersion();

    private native void pixy2USBLampOn();

    private native void pixy2USBLampOff();

    private native String pixy2USBGetBlocks();

    private native void pixy2USBTakePicture();

    private native void pixy2Calibrate();
}
