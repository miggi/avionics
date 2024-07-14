package org.bavovnar;

import com.diozero.util.SleepUtil;
import org.bavovnar.core.legacy.TimestampedData3f;
import org.bavovnar.device.MPU9255;
import org.bavovnar.navigation.UpdateListener;

import java.util.concurrent.TimeUnit;

public class Navigation implements Runnable, UpdateListener {

    private boolean stop;
    private boolean dataReady;
    private MPU9255 mpu;

    public Navigation(MPU9255 mpu9255) {
        this.stop = false;
        this.mpu = mpu9255;
        this.dataReady = false;

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
//        this.instruments = new Instruments();
    }


    @Override
    public void run() {
        TimestampedData3f accCorrected, gyroCorrected, magCorrected;
        while (!Thread.interrupted() && !stop) {

            SleepUtil.sleepMillis(mpu.getPollInterval());
            try {
                if (dataReady) {    //Store the latest data
                    dataReady = false;
                    instruments.setMagnetometer(mpu9250.getLatestGaussianData());        // #KW L492-501 done elsewhere, get the results
                    instruments.setAccelerometer(mpu9250.getLatestAcceleration());        // #KW L478-481 done elsewhere, get the results
                    instruments.setGyroscope(mpu9250.getLatestRotationalAcceleration());// #KW L485-488 done elsewhere, get the results
                    dataValid = true;
                }

                if (dataValid) // must have at least one value to startup calculations
                {
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

                    accCorrected = new TimestampedData3f(instruments.getAccelerometer());//preserve the timestamp set y & z
                    accCorrected.setX(-accCorrected.getX());                                //-ax

                    gyroCorrected = new TimestampedData3f(instruments.getGyroscope());    //preserve the timestamp
                    gyroCorrected.setX(gyroCorrected.getX() * (float) Math.PI / 180.0f);        //Pass gyro rate as rad/s
                    gyroCorrected.setY(-gyroCorrected.getY() * (float) Math.PI / 180.0f);        //-gy
                    gyroCorrected.setZ(-gyroCorrected.getZ() * (float) Math.PI / 180.0f);        //-gz

                    magCorrected = new TimestampedData3f(instruments.getMagnetometer()); //set timestamp and Z
                    float x = magCorrected.getX();
                    magCorrected.setX(magCorrected.getY());                                //swap X and Y, Z stays the same
                    //adjustedMag.setY(-adjustedMag.getX());
                    magCorrected.setY(-x);

                    instruments.updateInstruments(SensorFusion.MadgwickQuaternionUpdate(accCorrected, gyroCorrected, magCorrected, deltaTSec)); // #KW L921
//                    if (((float) nowNanoS - lastDisplayNanoS) / nanosPerSecf >= 1f / displayFrequencyHz) {
//                        lastDisplayNanoS = nowNanoS;
//						/*SystemLog.log(Navigate.class,SystemLog.LogLevel.USER_INFORMATION,"A " + mpu9250.getAvgAcceleration().toString()+
//								" G " + mpu9250.getAvgRotationalAcceleration().unStamp().toString()+
//								" M "  + mpu9250.getAvgGauss().unStamp().toString()+
//								" | Y,P&R: " + instruments.getAngles().toString());
//						SystemLog.log(Navigate.class,SystemLog.LogLevel.USER_INFORMATION, String.format(	" Freq: %5.1fHz %dk calcs%n",calculationFrequency,countDeltas/1000));*/
//                    }
                    for (UpdateListener listener : listeners) listener.dataUpdated();
                }
            } catch (InterruptedException e) {
                //close down signal
                stop = true;
            }
        }
    }

    @Override
    public void dataUpdated() {
        dataReady = true;
    }
}
