package org.bavovnar.navigation;

import com.diozero.util.SleepUtil;
import org.bavovnar.core.Imu;
import org.bavovnar.core.SensorData;
import org.bavovnar.core.legacy.TimestampedData3f;
import org.bavovnar.device.MPU9255;
import org.bavovnar.gateway.MQTTClient;

import java.util.concurrent.TimeUnit;

import static org.bavovnar.core.Utils.roundThreeDigits;
import static org.bavovnar.core.Utils.roundToFourDigits;

public class Navigation implements Runnable, UpdateListener {

    static private final float nanosPerSecf = ((float) TimeUnit.NANOSECONDS.convert(1, TimeUnit.SECONDS));
    private boolean stop;
    private boolean dataReady;
    private MPU9255 mpu;
    private Vehicle vehicle;
    private float deltaTSec;            // integration interval for both filter schemes time difference fractions of a second
    private long lastUpdateNanoS;        // used to calculate integration interval using nanotime
    private long nowNanoS;              // used to calculate integration interval using nanotime
    private float sumDeltas;            //Total time between calculations
    private int countDeltas;            //number of calculations
    private float calculationFrequency;    //calculation frequency in H
    private final MQTTClient mqttClient;

    public Navigation(MPU9255 mpu9255) {
        this.stop = false;
        this.mpu = mpu9255;
        this.dataReady = true;

        this.deltaTSec = 0.0f;
        this.sumDeltas = 0.0f;
        this.countDeltas = 0;
        this.nowNanoS = System.nanoTime();
        this.lastUpdateNanoS = nowNanoS;    //stop the first iteration having a massive delta
//        this.lastDisplayNanoS = nowNanoS;
//        this.displayFrequencyHz = 2;        //refresh the display every 1/2 a second

//        this.dataReady = false;
//        this.dataValid = false;
//        this.mpu9250 = mpu9250;
//        this.mpu9250.registerInterest(this);
//        this.deltaTSec = 0.0f;
//        this.sumDeltas = 0.0f;
//        this.countDeltas = 0;
//        this.nowNanoS = System.nanoTime();
//        this.lastUpdateNanoS = nowNanoS;    //stop the first iteration having a massive delta
//        this.lastDisplayNanoS = nowNanoS;
//        this.displayFrequencyHz = 2;        //refresh the display every 1/2 a second
//        this.listeners = new ArrayList<>();
        this.vehicle = new Vehicle();
        this.mqttClient = new MQTTClient();
    }


    @Override
    public void run() {
        TimestampedData3f accCorrected, gyroCorrected, magCorrected;
        Imu imuData = null;

        while (!Thread.interrupted() && !stop) {
            if (dataReady) {    //Store the latest data
                dataReady = false;

                imuData = mpu.getImuData();
                vehicle.setMagnetometer(new TimestampedData3f(imuData.getCompass()));
                vehicle.setAccelerometer(new TimestampedData3f(imuData.getAccel()));        // #KW L478-481 done elsewhere, get the results
                vehicle.setGyroscope(new TimestampedData3f(imuData.getGyro()));// #KW L485-488 done elsewhere, get the results
            }

            // new data or not recalulate the quaternion every 1 ms

            //Calculate integration interval
            nowNanoS = System.nanoTime();
            deltaTSec = ((float) nowNanoS - lastUpdateNanoS) / nanosPerSecf; // #KW L506
            lastUpdateNanoS = nowNanoS;                                    // #KW L507
            //calculate measurement frequency
            sumDeltas += deltaTSec;                                        // #KW L509
            countDeltas++;                                                // #KW L510
            calculationFrequency = countDeltas / sumDeltas;

            // #KW L512 Examples of calling the filters, READ BEFORE USING!!		!!!
            // sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
            // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
            // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
            // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
            // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
            // This is ok by aircraft orientation standards!
            // Pass gyro rate as rad/s
            // MadgwickQuaternionUpdate(-ax, ay, az, gx*PI/180.0f, -gy*PI/180.0f, -gz*PI/180.0f,  my,  -mx, mz); #KW L521
            accCorrected = new TimestampedData3f(vehicle.getAccelerometer());//preserve the timestamp set y & z
            accCorrected.setX(-accCorrected.getX());                                //-ax

            gyroCorrected = new TimestampedData3f(vehicle.getGyroscope());    //preserve the timestamp
            gyroCorrected.setX(gyroCorrected.getX() * (float) Math.PI / 180.0f);        //Pass gyro rate as rad/s
            gyroCorrected.setY(-gyroCorrected.getY() * (float) Math.PI / 180.0f);        //-gy
            gyroCorrected.setZ(-gyroCorrected.getZ() * (float) Math.PI / 180.0f);        //-gz

            magCorrected = new TimestampedData3f(vehicle.getMagnetometer()); //set timestamp and Z
            float x = magCorrected.getX();
            magCorrected.setX(magCorrected.getY());                                //swap X and Y, Z stays the same
            //adjustedMag.setY(-adjustedMag.getX());
            magCorrected.setY(-x);

            vehicle.updateVehicle(SensorFusion.MadgwickQuaternionUpdate(accCorrected, gyroCorrected, magCorrected, deltaTSec));
//                    if (((float) nowNanoS - lastDisplayNanoS) / nanosPerSecf >= 1f / displayFrequencyHz) {
//                        lastDisplayNanoS = nowNanoS;
//						/* SystemLog.log(Navigate.class,SystemLog.LogLevel.USER_INFORMATION,"A " + mpu9250.getAvgAcceleration().toString()+
//								" G " + mpu9250.getAvgRotationalAcceleration().unStamp().toString()+
//								" M "  + mpu9250.getAvgGauss().unStamp().toString()+
//								" | Y,P&R: " + instruments.getAngles().toString());
//						SystemLog.log(Navigate.class,SystemLog.LogLevel.USER_INFORMATION, String.format(	" Freq: %5.1fHz %dk calcs%n",calculationFrequency,countDeltas/1000));*/
//                    }

            if (imuData != null) {
                SensorData sensorData = new SensorData(
                        roundThreeDigits(imuData.getTemperature()),
                        roundThreeDigits((float) imuData.getAltitude()),
                        roundToFourDigits(gyroCorrected.toData3f()),
                        roundToFourDigits(accCorrected.toData3f()),
                        roundToFourDigits(magCorrected.toData3f()),
                        roundToFourDigits(vehicle.getQuaternion())
                );

                dataUpdated();
                try {
                    mqttClient.send(sensorData);
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
            SleepUtil.sleepMillis(mpu.getPollInterval());
        }
    }

    @Override
    public void dataUpdated() {
        dataReady = true;
    }
}
