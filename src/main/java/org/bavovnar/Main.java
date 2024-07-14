package org.bavovnar;

import org.bavovnar.device.MPU9255;
import org.bavovnar.navigation.Navigation;

public class Main {

    public static void main(String[] args) throws InterruptedException {
        MPU9255 mpu = new MPU9255();
//      mpu.calibrate();
        mpu.loadCalibrationData();

        Navigation nav = new Navigation(mpu);
        Thread navigationThread = new Thread(nav);
        navigationThread.start();

//        for (int i = 0; i < 100_000; i++) {
//            long startTime = System.currentTimeMillis();
//
//            Imu imuData = mpu.getImuData();
//            System.out.printf("Gyro = %s, Accel = %s, Magnet = %s, Temp = %f, Altitude = %f, time = %d ms]\n",
//                    imuData.getGyro().toString(),
//                    imuData.getAccel().toString(),
//                    imuData.getCompass().toString(),
//                    imuData.getTemperature(),
//                    imuData.getAltitude(),
//                    System.currentTimeMillis() - startTime
//            );
//
//            SleepUtil.sleepMillis(mpu.getPollInterval());
//            try {
//                mqttClient.send(imuData);
//            } catch (Exception e) {
//                throw new RuntimeException(e);
//            }
//        }
    }
}
