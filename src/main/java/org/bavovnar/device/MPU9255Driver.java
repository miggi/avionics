package org.bavovnar.device;

import com.diozero.devices.imu.invensense.*;
import org.bavovnar.device.constants.*;
import org.tinylog.Logger;

import com.diozero.api.DeviceInterface;
import com.diozero.api.I2CDevice;
import com.diozero.api.RuntimeIOException;
import com.diozero.util.SleepUtil;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@SuppressWarnings("unused")
public class MPU9255Driver implements DeviceInterface, MPU9255Constants, AK8963Constants {
    private static final byte AKM_DATA_READY = 0x01;
    private static final byte AKM_DATA_OVERRUN = 0x02;
    private static final byte AKM_OVERFLOW = (byte) 0x80;
    private static final byte AKM_DATA_ERROR = 0x40;

    private static final int MAX_PACKET_LENGTH = 12;
    private static final int MPU6050_TEMP_OFFSET = -521;
    private static final int MPU6500_TEMP_OFFSET = 0;
    private static final int MPU6050_TEMP_SENS = 340;
    private static final int MPU6500_TEMP_SENS = 321;
    private final AK8963MagScale magScale = AK8963MagScale.MFS_16BIT;
    private final AK8963MagMode magMode = AK8963MagMode.MM_100HZ;
    private GyroFullScaleRange gyroScaleRange;
    /* Matches accel_cfg >> 3 & 0x03 */
    private AccelFullScaleRange accel_fsr;
    /* Sample rate, NOT rate divider. */
    private int sample_rate;
    /* Matches int enable register. */
    private boolean int_enable;

    /* Sampling rate used when DMP is enabled. */
    private int magnetometerSampleRate;

    private byte compass_addr;
    private AK8975Driver magSensor;
    private final I2CDevice i2cDevice;

    private final I2CDevice i2cDeviceAK8963;

    private float[] magFactoryCalibration;
    private float[] magCalibration = new float[]{0.0f, 0.0f, 0.0f};
    private float[] gyroCalibration = new float[]{0.0f, 0.0f, 0.0f};
    private float[] accCalibration = new float[]{0.0f, 0.0f, 0.0f};

    /**
     * Default constructor, uses default I2C address.
     *
     * @param controller I2C controller
     * @throws RuntimeIOException if an I/O error occurs
     */
    public MPU9255Driver(int controller) throws RuntimeIOException {
        this(controller, MPU9255_ADDRESS, AK8963_ADDRESS);
    }

    /**
     * Specific address constructor.
     *
     * @param controller I2C controller
     * @param devAddress address I2C address
     * @throws RuntimeIOException if an I/O error occurs
     */
    public MPU9255Driver(int controller, int devAddress, int magAddress) throws RuntimeIOException {
        i2cDevice = I2CDevice.builder(devAddress).setController(controller).build();
        i2cDeviceAK8963 = I2CDevice.builder(magAddress).setController(controller).build();
    }

    @Override
    public void close() throws RuntimeIOException {
        if (magSensor != null) {
            magSensor.close();
        }
        i2cDevice.close();
    }

    /**
     * Initialize hardware. Initial configuration:
     * <p>
     * Gyro FSR: +/- 250DPS Accel FSR +/- 2G
     * Data ready interrupt: Disabled, active low, unlatched.
     *
     * @throws RuntimeIOException if an I/O error occurs
     */
    public void init() throws RuntimeIOException {
        // wake up device, Clear sleep bits bit (6), enable all sensors
        i2cDevice.writeByteData(PWR_MGMT_1, 0x80);
        SleepUtil.sleepMillis(100); // Wait for all registers to reset

        // get stable time source
        i2cDevice.writeByteData(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
        SleepUtil.sleepMillis(200);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250_Pi4j, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        i2cDevice.writeByteData(CONFIG, MPU9255DLPBandwidth.F01BW0184.bits);//set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        i2cDevice.writeByteData(SMPLRT_DIV, MPU9255SampleRateDiv.HZ200.bits);
        // Use a 200 Hz rate; a rate consistent with the filter update rate
        // determined inset in CONFIG above

        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the device bus and all can be controlled by the Arduino as master
        //ro.writeByteRegister(Registers.INT_PIN_CFG.getValue(), (byte)0x12);  // INT is 50 microsecond pulse and any read to clear

        i2cDevice.writeByteData(INT_PIN_CFG, (byte) 0x22);  // INT is 50 microsecond pulse and any read to clear - as per MPUBASICAHRS_T3
        i2cDevice.writeByteData(INT_ENABLE, (byte) 0x01);  // Enable data ready (bit 0) interrupt

        /* Set to invalid values to ensure no I2C writes are skipped. */
        gyroScaleRange = null;
        accel_fsr = null;
        sample_rate = 0xFFFF;
        magnetometerSampleRate = 0xFFFF;

        // memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
        setGyroFullScaleRange(GyroFullScaleRange.INV_FSR_500DPS);
        setAccelerometerScaleRange(AccelFullScaleRange.INV_FSR_16G);

        setSampleRate(100);
        setupMagnetometer();
    }

    /**
     * setCalibrationMode9250 - puts the device into calibrate mode
     *
     * @throws InterruptedException - If sleep was interrupted
     */
    public void setCalibrationMode() throws InterruptedException {
        Logger.info("Setting calibration mode.");

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        i2cDevice.writeByteData(PWR_MGMT_1, (byte) 0x01);
        i2cDevice.writeByteData(PWR_MGMT_2, (byte) 0x00); //ALL Enabled
        Thread.sleep(200);

        // Configure device for bias calculation
        i2cDevice.writeByteData(INT_ENABLE, (byte) 0x00);   // Disable all interrupts
        i2cDevice.writeByteData(FIFO_EN, (byte) 0x00);      // Disable FIFO
        i2cDevice.writeByteData(PWR_MGMT_1, (byte) 0x01);   // Turn on internal clock source
        i2cDevice.writeByteData(I2C_MST_CTRL, (byte) 0x00); // Disable device master
        //roMPU.writeByteRegister(Registers.USER_CTRL,(byte) 0x00);    // Disable FIFO and device master modes
        //Thread.sleep(20);
        i2cDevice.writeByteData(USER_CTRL, (byte) 0x0C);    // Reset FIFO and DMP NB the 0x08 bit is the DMP shown as reserved in docs

        Thread.sleep(15);

        i2cDevice.writeByteData(CONFIG, (byte) 1);       // Set low-pass filter to 188 Hz
        i2cDevice.writeByteData(SMPLRT_DIV, (byte) 0);   // Set sample rate to 1 kHz = Internal_Sample_Rate / (1 + SMPLRT_DIV)
    }

    public float[] calibrateGyroscope(int samples) {
        Logger.info("********* Gyroscope calibration. Samples count = " + samples);
        // Assumes we are in calibration bits via setCalibrationMode9250();
        // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        i2cDevice.writeByteData(GYRO_CONFIG, GyroFullScaleRange.INV_FSR_250DPS.getBitVal());

        List<short[]> readings = new ArrayList<>();
        for (int i = 0; i < samples; i++) {
            ByteBuffer data = i2cDevice.readI2CBlockDataByteBuffer(GYRO_XOUT_H, 6);

            short x = data.getShort();
            short y = data.getShort();
            short z = data.getShort();
            readings.add(new short[]{x, y, z});
            SleepUtil.sleepMillis(5);
        }

        int[] gyroBiasSum = new int[]{0, 0, 0}; //32 bit to allow for accumulation without overflow

        for (short[] reading : readings) {
            gyroBiasSum[0] += reading[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            gyroBiasSum[1] += reading[1];
            gyroBiasSum[2] += reading[2];
        }

        Logger.info("Gyro bias sum: " + Arrays.toString(gyroBiasSum)
                + String.format(" [0x%X, 0x%X, 0x%X]", gyroBiasSum[0], gyroBiasSum[1], gyroBiasSum[2]));

        //calculate averages
        short[] gyroBiasAvg = new short[]{0, 0, 0}; //16 bit average
        if (readings.size() > 0) {
            gyroBiasAvg[0] = (short) ((gyroBiasSum[0] / samples) & 0xffff); //mask out any sign extension
            gyroBiasAvg[1] = (short) ((gyroBiasSum[1] / samples) & 0xffff);
            gyroBiasAvg[2] = (short) ((gyroBiasSum[2] / samples) & 0xffff);
        }
        Logger.info("Gyro Bias average: " + Arrays.toString(gyroBiasAvg) +
                String.format(" [0x%X, 0x%X, 0x%X]", gyroBiasAvg[0], gyroBiasAvg[1], gyroBiasAvg[2]));

        this.gyroCalibration = new float[]{
                (float) gyroBiasAvg[0],
                (float) gyroBiasAvg[1],
                (float) gyroBiasAvg[2]
        };

        Logger.info("********* End process of Gyro calibration. Values = " + Arrays.toString(gyroCalibration));

        return gyroCalibration;
    }

    public float[] calibrateAccelerometer(int samples) {
        // part of accel gyro cal MPU9250 in Kris Winer code - this code is only the Accelerometer elements
        Logger.info("********* Accelerometer calibration. Samples count = " + samples);
        i2cDevice.writeByteData(ACCEL_CONFIG, MPU9255AccScale.AFS_16G.bits); // Set accelerometer full-scale to 16 g, maximum sensitivity

        List<short[]> readings = new ArrayList<>();
        for (int i = 0; i < samples; i++) {

            ByteBuffer data = i2cDevice.readI2CBlockDataByteBuffer(ACCEL_XOUT_H, 6); //TODO: need review
            short x = data.getShort();
            short y = data.getShort();
            short z = data.getShort();
            readings.add(new short[]{x, y, z});

            SleepUtil.sleepMillis(2);
        }

        int[] accelBiasSum = new int[]{0, 0, 0}; //32 bit to allow for accumulation without overflow
        for (short[] reading : readings) //#KW L962
        {
            accelBiasSum[0] += reading[0];        //#KW L972
            accelBiasSum[1] += reading[1];    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accelBiasSum[2] += reading[2];
        }
        Logger.info("Accel Bias sum: " + Arrays.toString(accelBiasSum)
                + String.format(" [0x%X, 0x%X, 0x%X]", accelBiasSum[0], accelBiasSum[1], accelBiasSum[2]));

        //calculate averages
        short[] accelBiasAvg = new short[]{0, 0, 0}; //16 bit average
        accelBiasAvg[0] = (short) ((accelBiasSum[0] / samples) & 0xffff); // #KW L980
        accelBiasAvg[1] = (short) ((accelBiasSum[1] / samples) & 0xffff); // Normalise sums to get average count biases
        accelBiasAvg[2] = (short) ((accelBiasSum[2] / samples) & 0xffff);

        if (accelBiasAvg[0] > 0)
            accelBiasAvg[0] -= accel_fsr.getSensitivityScaleFactor(); // #KW 987 Remove gravity from the x-axis. Assume rocket in vertacal state
        else accelBiasAvg[0] += accel_fsr.getSensitivityScaleFactor();

        Logger.info("Accel bias average: " + Arrays.toString(accelBiasAvg) +
                String.format(" [0x%X, 0x%X, 0x%X]", accelBiasAvg[0], accelBiasAvg[1], accelBiasAvg[2]));

//        this.setDeviceBias(new Data3f( 	(float) accelBiasAvg [0] / 2.0f / (float)accelSensitivity,
//                (float) accelBiasAvg [1] / 2.0f / (float)accelSensitivity,
//                (float) accelBiasAvg [2] / 2.0f / (float)accelSensitivity)

        this.accCalibration = new float[]{
                (float) accelBiasAvg[0],
                (float) accelBiasAvg[1],
                (float) accelBiasAvg[2]
        };

        Logger.info("********* End Accelerometer calibration. Values = " + Arrays.toString(accCalibration));

        return accCalibration;
    }

    /**
     * Set the gyro full-scale range.
     *
     * @param fsr Desired full-scale range.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public void setGyroFullScaleRange(GyroFullScaleRange fsr) throws RuntimeIOException {
        if (gyroScaleRange == fsr) {
            return;
        }

        Logger.info("Setting gyro config to 0x%x%n", fsr.getBitVal());
        // gyro_cfg == 0x1B == MPU9150_RA_GYRO_CONFIG
        i2cDevice.writeByteData(GYRO_CONFIG, fsr.getBitVal());
        gyroScaleRange = fsr;
    }

    /**
     * Read raw gyro data directly from the registers.
     *
     * @return Raw data in hardware units.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public float[] readGyroscope() throws RuntimeIOException {
        // raw_gyro == 0x43 == MPU9150_RA_GYRO_XOUT_H
        ByteBuffer buffer = i2cDevice.readI2CBlockDataByteBuffer(GYRO_XOUT_H, 6);
        short x = buffer.getShort();
        short y = buffer.getShort();
        short z = buffer.getShort();

        return new float[]{
                y - gyroCalibration[0], // NOTE: Swapped values Y and X
                x - gyroCalibration[1],
                z - gyroCalibration[2]
        };
    }

    /**
     * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS (Register
     * 28). For each full scale setting, the accelerometers' sensitivity per LSB in ACCEL_xOUT
     * is shown in the table below. AFS_SEL Full Scale Range LSB Sensitivity 0 +/-2g 16384
     * LSB/mg 1 +/-4g 8192 LSB/mg 2 +/-8g 4096 LSB/mg 3 +/-16g 2048 LSB/mg
     *
     * @return Raw data in hardware units.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public float[] readAccelerometer() throws RuntimeIOException {
        ByteBuffer buffer = i2cDevice.readI2CBlockDataByteBuffer(ACCEL_XOUT_H, 6);

        short x = buffer.getShort();
        short y = buffer.getShort();
        short z = buffer.getShort();

        return new float[]{
                (x - accCalibration[0]),
                (y - accCalibration[1]),
                (z - accCalibration[2])
        };
    }

    /**
     * Read temperature data directly from the registers. The scale factor and offset for the
     * temperature sensor are found in the Electrical Specifications table in the MPU-9255
     * Product Specification document. The temperature in degrees C for a given register value
     * may be computed as: Temperature in degrees C = (TEMP_OUT Register Value as a signed
     * quantity)/340 + 35 Please note that the math in the above equation is in decimal.
     *
     * @return Temperature
     * @throws RuntimeIOException if an I/O error occurs
     */
    public float readTemperature() throws RuntimeIOException {
        // temp == 0x41 == TEMP_OUT_H
        short raw = i2cDevice.readShort(TEMP_OUT_H);
        // raw = (tmp[0] << 8) | (tmp[1] & 0xff);

        // float temperatureC = rawTemp / 333.87 + 21.0;
        return ((raw) / 333.87f) + 15f;
    }


    /**
     * Set the accel full-scale range.
     *
     * @param fsr Desired full-scale range.
     * @return status
     * @throws RuntimeIOException if an I/O error occurs
     */
    public boolean setAccelerometerScaleRange(AccelFullScaleRange fsr) throws RuntimeIOException {
        if (accel_fsr == fsr) {
            return true;
        }

        System.out.format("Setting accel config to 0x%x%n", fsr.getBitVal());
        // accel_cfg == 0x1C == MPU9150_RA_ACCEL_CONFIG
        i2cDevice.writeByteData(ACCEL_CONFIG, fsr.getBitVal());
        accel_fsr = fsr;

        return true;
    }

    public int getSampleRate() {
        return sample_rate;
    }

    /**
     * Set sampling rate. Sampling rate must be between 4Hz and 1kHz.
     *
     * @param rate Desired sampling rate (Hz).
     * @return status
     * @throws RuntimeIOException if an I/O error occurs
     */
    public boolean setSampleRate(int rate) throws RuntimeIOException {
        Logger.debug("mpu_set_sample_rate({})", rate);

        int new_rate = rate;
        if (new_rate < 4) {
            new_rate = 4;
        } else if (new_rate > 1000) {
            new_rate = 1000;
        }

        int data = 1000 / new_rate - 1;
        Logger.debug("Setting sample rate to {}", data);
        // rate_div == 0x19 == MPU9150_RA_SMPL_RATE_DIV
        i2cDevice.writeByteData(SMPLRT_DIV, data);

        sample_rate = 1000 / (1 + data);
        setMagnetometerSampleRate(Math.min(magnetometerSampleRate, MAX_COMPASS_SAMPLE_RATE));

        Logger.debug("Automatically setting LPF to {}", sample_rate >> 1);
        /* Automatically set LPF to 1/2 sampling rate. */
//        mpu_set_lpf(sample_rate >> 1);  TODO:

        return true;
    }

    public int getMagnetometerSampleRate() {
        return magnetometerSampleRate;
    }

    /**
     * Set compass sampling rate. The compass on the auxiliary I2C bus is read by the MPU
     * hardware at a maximum of 100Hz. The actual rate can be set to a fraction of the gyro
     * sampling rate.
     * <p>
     * WARNING: The new rate may be different than what was requested. Call
     * mpu_get_compass_sample_rate to check the actual setting.
     *
     * @param rate Desired compass sampling rate (Hz).
     * @throws RuntimeIOException if an I/O error occurs
     */
    public void setMagnetometerSampleRate(int rate) throws RuntimeIOException {
        Logger.debug("setMagnetometerSampleRate({})", rate);
        if (rate == 0 || rate > sample_rate || rate > MAX_COMPASS_SAMPLE_RATE) {
            return;
        }

        byte div = (byte) (sample_rate / rate - 1);
        // s4_ctrl == 0x34 == MPU9150_RA_I2C_SLV4_CTRL
        i2cDeviceAK8963.writeByteData(I2C_SLV4_CTRL, div);
        magnetometerSampleRate = sample_rate / (div + 1);
    }

    /**
     * Get gyro sensitivity scale factor.
     *
     * @return Sensitivy, conversion from hardware units to dps.
     */
    public double getGyroScale() {
        /*
         * float sens; switch (gyro_fsr) { case INV_FSR_250DPS: sens = 131.0f; break; case
         * INV_FSR_500DPS: sens = 65.5f; break; case INV_FSR_1000DPS: sens = 32.8f; break; case
         * INV_FSR_2000DPS: sens = 16.4f; break; default: sens = -1f; }
         *
         * return sens;
         */
        return gyroScaleRange.getScale();
    }

    /**
     * Get accel sensitivity scale factor.
     *
     * @return Sensitivity. Conversion from hardware units to g's.
     */
    public double getAccelerometerScale() {
        /*
         * int sens; switch (accel_fsr) { case INV_FSR_2G: sens = 16384; break; case INV_FSR_4G:
         * sens = 8192; break; case INV_FSR_8G: sens = 4096; break; case INV_FSR_16G: sens = 2048;
         * break; default: return -1; }
         */
        return accel_fsr.getScale();
    }

    public float getMagnetometerScale() {
        return magScale.res;
    }

    public void setupMagnetometer() throws RuntimeIOException {
        Logger.debug("Configure Magnetometer AK8963");

        // First extract the factory calibration for each magnetometer axis
        i2cDeviceAK8963.writeByteData(AK8963_CNTL1, (byte) 0x00); // #KW 836 Power down magnetometer
        SleepUtil.sleepMillis(10);

        i2cDeviceAK8963.writeByteData(AK8963_CNTL1, (byte) 0x0F); // #KW 838 Enter Fuse ROM access bits
        SleepUtil.sleepMillis(10);

        byte[] calibrationValues = i2cDeviceAK8963.readI2CBlockDataByteArray(AK8963_ASAX, 3);  // Read the x-, y-, and z-axis calibration values
        this.magFactoryCalibration = new float[]{
                ((float) (calibrationValues[0] - 128)) / 256f + 1f,   // #KW 841-843 Return x-axis sensitivity adjustment values, etc.
                ((float) (calibrationValues[1] - 128)) / 256f + 1f,
                ((float) (calibrationValues[2] - 128)) / 256f + 1f
        };

        i2cDeviceAK8963.writeByteData(AK8963_CNTL1, (byte) 0x00); // #KW 844 Power down magnetometer
        SleepUtil.sleepMillis(10);

        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL1 register,
        // and enable continuous bits data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        // set to MagScale.MFS_16BIT.bits and MagMode.MM_100HZ set as final lines 48 & 49. register write should be 0x16

        i2cDeviceAK8963.writeByteData(AK8963_CNTL1, (byte) (magScale.bits | magMode.bits)); // #KW 849 Set magnetometer data resolution and sample ODR ####16bit already shifted
        SleepUtil.sleepMillis(10);
    }

    /**
     * Read calibrated compass data.
     *
     * @return data Raw data in hardware units.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public float[] readMagnetometer() throws RuntimeIOException {
        byte dataReady = (byte) (i2cDeviceAK8963.readByteData(AK8963_ST1) & 0x01); //DRDY - Data ready bit0 1 = data is ready
        if (dataReady == 0) throw new RuntimeIOException("Not ready");

        float magX, magY, magZ;

        // #KW 494 readMagData - data is ready, read it NB bug fix here read was starting from ST1 not XOUT_L
        byte[] buffer = i2cDeviceAK8963.readI2CBlockDataByteArray(AK8963_XOUT_L, 7); // #KW L815 6 data bytes x,y,z 16 bits stored as little Endian (L/H)
        // Check if magnetic sensor overflow set, if not then report data
        //roAK.readByteRegister(Registers.AK8963_ST2);// Data overflow bit 3 and data read error status bit 2

        byte status2 = buffer[6]; // Status2 register must be read as part of data read to show device data has been read
        if ((status2 & 0x08) == 0) //#KW 817 bit3 HOFL: Magnetic sensor overflow is normal (no Overflow), data is valid
        {   //#KW L818-820
            magX = (short) ((buffer[1] << 8) | (buffer[0] & 0xFF)); // Turn the MSB and LSB into a signed 16-bit value
            magY = (short) ((buffer[3] << 8) | (buffer[2] & 0xFF)); // Data stored as little Endian
            magZ = (short) ((buffer[5] << 8) | (buffer[4] & 0xFF)); // mask to prevent sign extension in LSB (bug fix)

            //the stored calibration results is applied here as there is no hardware correction stored in the hardware via calibration
            //#KW L496-L501. scale() does the multiplication by magScale L499-501

            return new float[]{
                    (magX * magFactoryCalibration[0] - getMagCalibration()[0]),
                    (magY * magFactoryCalibration[1] - getMagCalibration()[1]),
                    (magZ * magFactoryCalibration[2] - getMagCalibration()[2])
            };
        }

        return new float[]{0, 0, 0}; //TODO: check if it's correct behaviour
    }

    public float[] calibrateMagnetometer() throws InterruptedException {
        Logger.info("********* Calibrating Magnetometer AK8963");

        float[] bias = {0.0f, 0.0f, 0.0f}, scale = {0, 0, 0};
        float[] max = {(short) -32767, (short) -32767, (short) -32767},
                min = {(short) 32767, (short) 32767, (short) 32767},
                temp = {0, 0, 0};

        Logger.info("Magnetometer Calibration: Wave device in a figure '8' until done!");
        // #KW L1073 shoot for ~fifteen seconds of mag data
        for (int i = 0; i < magMode.sampleCount; i++) {
            float[] magn = readMagnetometer();

            temp[0] = magn[0];
            temp[1] = magn[1];
            temp[2] = magn[2];

            for (int j = 0; j < 3; j++) {
                if (temp[j] > max[j]) max[j] = temp[j];
                if (temp[j] < min[j]) min[j] = temp[j];
            }
            if (magMode == AK8963MagMode.MM_8HZ)
                SleepUtil.sleepMillis(135);  // at 8 Hz ODR, new mag data is available every 125 ms
            if (magMode == AK8963MagMode.MM_100HZ)
                SleepUtil.sleepMillis(10);  // at 100 Hz ODR, new mag data is available every 10 ms
        }

        bias[0] = (max[0] + min[0]) / 2;  // get average x mag bias in counts
        bias[1] = (max[1] + min[1]) / 2;  // get average y mag bias in counts
        bias[2] = (max[2] + min[2]) / 2;  // get average z mag bias in counts

        this.magCalibration = new float[]{
                bias[0] * magFactoryCalibration[0],
                bias[1] * magFactoryCalibration[1],
                bias[2] * magFactoryCalibration[2]
        };

        // #KW L1099 Get soft iron correction estimate
        scale[0] = (max[0] - min[0]) / 2;  // get average x axis max chord length in counts
        scale[1] = (max[1] - min[1]) / 2;  // get average y axis max chord length in counts
        scale[2] = (max[2] - min[2]) / 2;  // get average z axis max chord length in counts

        float avgRad = (scale[0] + scale[1] + scale[2]) / 3.0f; // #KW L1104-5

        Logger.info("Magnetometer Biases (hard): " + Arrays.toString(bias));
        Logger.info("Magnetometer Biases (soft): " + Arrays.toString(scale));

//        this.setDeviceScaling(new Data3f(avgRad / ((float) scale[0]), // #KW L1107-9 save mag scale for main program
//                avgRad / ((float) scale[1]), // deviceScale was pass by ref dest2 in Kris Winer code
//                avgRad / ((float) scale[2])));

        Logger.info("****** Magnetometer calibration finished. Values = " + Arrays.toString(magCalibration));

        return this.magCalibration;
    }

    public void setMagCalibration(float[] magCalibration) {
        this.magCalibration = magCalibration;
    }

    public void setGyroCalibration(float[] gyroCalibration) {
        this.gyroCalibration = gyroCalibration;
    }

    public void setAccCalibration(float[] accCalibration) {
        this.accCalibration = accCalibration;
    }

    public float[] getMagCalibration() {
        return magCalibration;
    }

    public float[] getGyroCalibration() {
        return gyroCalibration;
    }

    public float[] getAccCalibration() {
        return accCalibration;
    }
}
