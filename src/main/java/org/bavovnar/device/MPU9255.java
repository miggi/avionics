package org.bavovnar.device;

import com.diozero.api.I2CConstants;
import com.diozero.api.RuntimeIOException;
import com.diozero.devices.BMx280;
import com.diozero.devices.imu.*;
import org.bavovnar.core.Imu;
import org.bavovnar.core.ImuFactory;
import org.bavovnar.core.Utils;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.tinylog.Logger;

import java.util.Arrays;

public class MPU9255 implements ImuInterface {
    private static final String DEVICE_NAME = "MPU-9255";
    private static final String SETTINGS_DIR = "./.av";
    private static final String CALIBRATION_PATH = SETTINGS_DIR + "/calibration";
    private static final int BMP280_ID = 0x77;
    private MPU9255Driver driver;
    private BMx280 bmx280;
    public MPU9255() throws RuntimeIOException {
        this(I2CConstants.CONTROLLER_0);
    }

    public MPU9255(int busId) throws RuntimeIOException {
        driver = new MPU9255Driver(busId);
        driver.init();
        bmx280 = BMx280.I2CBuilder
                .builder(busId)
                .setAddress(BMP280_ID)
                .build();
    }

    @Override
    public Imu getImuData() throws RuntimeIOException {
        return ImuFactory.newInstance(
                driver.readGyroscope(),
                driver.readAccelerometer(),
                driver.readMagnetometer(),
                Utils.roundThreeDigits(driver.readTemperature()),
                Utils.roundThreeDigits(getAltitude(bmx280.getPressure())),
                driver.getGyroScale(),
                driver.getAccelerometerScale(),
                driver.getMagnetometerScale());
    }

    @Override
    public boolean hasGyro() {
        return true;
    }

    @Override
    public boolean hasAccelerometer() {
        return true;
    }

    @Override
    public boolean hasCompass() {
        return true;
    }

    public void calibrate() throws InterruptedException {
        driver.setCalibrationMode();

        int samples = 1000;
        float[] gyro = driver.calibrateGyroscope(samples);
        float[] acc = driver.calibrateAccelerometer(samples);
        float[] mag = driver.calibrateMagnetometer();

        Logger.info("****** Saving calibration data");
        Utils.createDirectory(SETTINGS_DIR);
        Utils.createFile(CALIBRATION_PATH,
                "g:" + Utils.toString(gyro) +
                "\na:" + Utils.toString(acc) +
                "\nm:" + Utils.toString(mag)
        );
    }
    public void loadCalibrationData() {
        driver.setGyroCalibration(Utils.readFromFile("g:", CALIBRATION_PATH));
        driver.setAccCalibration(Utils.readFromFile("a:", CALIBRATION_PATH));
        driver.setMagCalibration(Utils.readFromFile("m:", CALIBRATION_PATH));

        Logger.info("Loaded calibration data " + String.format("gyro = %s, acc = %s, mag = %s",
                Arrays.toString(driver.getGyroCalibration()),
                Arrays.toString(driver.getAccCalibration()),
                Arrays.toString(driver.getMagCalibration())));
    }

    @Override
    public void close() throws RuntimeIOException {
        driver.close();
    }

    @Override
    public String getImuName() {
        return DEVICE_NAME;
    }

    @Override
    public Vector3D getGyroData() throws RuntimeIOException {
        return ImuFactory.createVector(driver.readGyroscope(), driver.getGyroScale());
    }

    @Override
    public Vector3D getAccelerometerData() throws RuntimeIOException {
        return ImuFactory.createVector(driver.readAccelerometer(), driver.getAccelerometerScale());
    }

    @Override
    public Vector3D getCompassData() throws RuntimeIOException {
        return ImuFactory.createVector(driver.readMagnetometer(), driver.getMagnetometerScale());
    }
    @Override
    public int getPollInterval() {
        return 10; // milliseconds
    }

    @Override
    public void startRead() {}

    @Override
    public void stopRead() {}

    /**
     * @return Return altitude in meters
     */
    private static double getAltitude(float pressure) {
        final int MEAN_SEA_LEVEL_PRESSURE = 1013;

        return 44330.0 * (1.0 - Math.pow(pressure / (MEAN_SEA_LEVEL_PRESSURE), 0.1902949571));
    }

    @Override
    public void addTapListener(TapListener listener) {}

    @Override
    public void addOrientationListener(OrientationListener listener) {}
}
