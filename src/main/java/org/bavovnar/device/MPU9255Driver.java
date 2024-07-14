package org.bavovnar.device;

import com.diozero.devices.imu.invensense.*;
import org.bavovnar.device.constants.AK8963Constants;
import org.bavovnar.device.constants.IMU9255Constants;
import org.tinylog.Logger;

import com.diozero.api.DeviceInterface;
import com.diozero.api.I2CDevice;
import com.diozero.api.RuntimeIOException;
import com.diozero.util.SleepUtil;

import java.nio.ByteBuffer;

@SuppressWarnings("unused")
public class IMU9255Driver implements DeviceInterface, IMU9255Constants, AK8963Constants {
    private static final byte AKM_DATA_READY = 0x01;
    private static final byte AKM_DATA_OVERRUN = 0x02;
    private static final byte AKM_OVERFLOW = (byte) 0x80;
    private static final byte AKM_DATA_ERROR = 0x40;

    private static final int MAX_PACKET_LENGTH = 12;
    private static final int MPU6050_TEMP_OFFSET = -521;
    private static final int MPU6500_TEMP_OFFSET = 0;
    private static final int MAX_FIFO = 1024;
    private static final int MPU6050_TEMP_SENS = 340;
    private static final int MPU6500_TEMP_SENS = 321;

    private int devAddress;
    /* Matches gyro_cfg >> 3 & 0x03 */
    private GyroFullScaleRange gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    private AccelFullScaleRange accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    private byte sensors;
    private LowPassFilter lpf;
    private ClockSource clk_src;
    /* Sample rate, NOT rate divider. */
    private int sample_rate;
    /* Matches fifo_en register. */
    private byte fifo_enable;
    /* Matches int enable register. */
    private boolean int_enable;
    /* true if devices on auxiliary I2C bus appear on the primary. */
    private Boolean bypass_mode;
    /*
     * true if half-sensitivity. NOTE: This doesn't belong here, but everything else in hw_s
     * is const, and this allows us to save some precious RAM.
     */
    private boolean lp_accel_mode;
    /* true if interrupts are only triggered on motion events. */
    private boolean int_motion_only;
    // struct motion_int_cache_s cache;
    /* true for active low interrupts. */
    private boolean active_low_int;
    /* true for latched interrupts. */
    private boolean latched_int;
    /* true if DMP is enabled. */
    private boolean dmp_on;
    /* Ensures that DMP will only be loaded once. */
    private boolean dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    private int dmp_sample_rate;
    /* Compass sample rate. */
    private int compass_sample_rate;
    private byte compass_addr;
    private AK8975Driver magSensor;
    private I2CDevice i2cDevice;

    private float[] magCalibration;

    private float[] gyroCalibration;

    private float[] accCalibration;

    /**
     * Default constructor, uses default I2C address.
     *
     * @throws RuntimeIOException if an I/O error occurs
     * @param controller I2C controller
     */
    public IMU9255Driver(int controller) throws RuntimeIOException {
        this(controller, MPU9255_ADDRESS);
    }

    /**
     * Specific address constructor.
     *
     * @param controller I2C controller
     * @param devAddress address I2C address
     * @throws RuntimeIOException if an I/O error occurs
     */
    public IMU9255Driver(int controller, int devAddress) throws RuntimeIOException {
        i2cDevice = I2CDevice.builder(devAddress).setController(controller).build();
        this.devAddress = devAddress;
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
     *
     *   Gyro FSR: +/- 2000DPS Accel FSR +/- 2G
     *   DLPF: 42Hz
     *   FIFO rate: 50Hz
     *   Clock source: Gyro PLL FIFO: Disabled.
     *   Data ready interrupt: Disabled, active low, unlatched.
     *
     * @throws RuntimeIOException if an I/O error occurs
     */
    public void mpuInit() throws RuntimeIOException {
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
        i2cDevice.writeByteData(CONFIG, GT_DLPF.F01BW0041.bits);//set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        i2cDevice.writeByteData(SMPLRT_DIV, SampleRateDiv.HZ200.bits);  // Use a 200 Hz rate; a rate consistent with the filter update rate
        // determined inset in CONFIG above

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the device bus and all can be controlled by the Arduino as master
        //ro.writeByteRegister(Registers.INT_PIN_CFG.getValue(), (byte)0x12);  // INT is 50 microsecond pulse and any read to clear
        i2cDevice.writeByteData(INT_PIN_CFG, (byte)0x22);  // INT is 50 microsecond pulse and any read to clear - as per MPUBASICAHRS_T3
        i2cDevice.writeByteData(INT_ENABLE, (byte)0x01);  // Enable data ready (bit 0) interrupt

        /* Set to invalid values to ensure no I2C writes are skipped. */
        sensors = (byte) 0xFF;
        gyro_fsr = null;
        accel_fsr = null;
        lpf = null;
        sample_rate = 0xFFFF;
        fifo_enable = (byte) 0xFF;
        bypass_mode = null;
        compass_sample_rate = 0xFFFF;
        /* mpu_set_sensors always preserves this setting. */
        clk_src = ClockSource.INV_CLK_PLL;
        /* Handled in next call to mpu_set_bypass. */
        active_low_int = true;
        latched_int = false;
        int_motion_only = false;
        lp_accel_mode = false;
        // memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache));
        dmp_on = false;
        dmp_loaded = false;
        dmp_sample_rate = 0;

        setGyroFullScaleRange(GyroFullScaleRange.INV_FSR_2000DPS);
        mpu_set_accel_fsr(AccelFullScaleRange.INV_FSR_2G);
        setSampleRate(50);

        // if (int_param)
        // reg_int_cb(int_param);

        setupCompass();
        mpuSetCompassSampleRate(10);

    }

    /**
     * Read raw gyro data directly from the registers.
     *
     * @return Raw data in hardware units.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public short[] getGyroscopeData() throws RuntimeIOException {

        // raw_gyro == 0x43 == MPU9150_RA_GYRO_XOUT_H
        ByteBuffer buffer = i2cDevice.readI2CBlockDataByteBuffer(GYRO_XOUT_H, 6);
        short x = buffer.getShort();
        short y = buffer.getShort();
        short z = buffer.getShort();
        Logger.info("gyro reg values = (%d, %d, %d)%n", Short.valueOf(x), Short.valueOf(y), Short.valueOf(z));
        /*
         * byte[] data = readBytes(MPU9150_RA_GYRO_XOUT_H, 6) short x = (short)((data[0] << 8) |
         * (data[1] & 0xff)); short y = (short)((data[2] << 8) | (data[3] & 0xff)); short z =
         * (short)((data[4] << 8) | (data[5] & 0xff));
         */

        return new short[] { x, y, z };
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
    public short[] mpu_get_accel_reg() throws RuntimeIOException {
        ByteBuffer buffer = i2cDevice.readI2CBlockDataByteBuffer(ACCEL_XOUT_H, 6);
        short x = buffer.getShort();
        short y = buffer.getShort();
        short z = buffer.getShort();
		/*-
		byte[] data = readBytes(MPU9150_RA_ACCEL_XOUT_H, 6);
		short x = (short)((data[0] << 8) | (data[1] & 0xff));
		short y = (short)((data[2] << 8) | (data[3] & 0xff));
		short z = (short)((data[4] << 8) | (data[5] & 0xff));
		*/

        return new short[] { x, y, z };
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
    public float getTemperature() throws RuntimeIOException {
        if (sensors == 0) {
            return -1;
        }
        // temp == 0x41 == TEMP_OUT_H
        short raw = i2cDevice.readShort(TEMP_OUT_H);
        // raw = (tmp[0] << 8) | (tmp[1] & 0xff);

        float val = ((raw - MPU6050_TEMP_OFFSET) / (float) MPU6050_TEMP_SENS) + 35;
        // float temperatureC = rawTemp / 333.87 + 21.0;
        return val;
    }

    /**
     * Get the gyro full-scale range.
     *
     * @return fsr Current full-scale range.
     */
    public GyroFullScaleRange mpu_get_gyro_fsr() {
        return gyro_fsr;
    }

    /**
     * Set the gyro full-scale range.
     *
     * @param fsr Desired full-scale range.
     * @return status
     * @throws RuntimeIOException if an I/O error occurs
     */
    public boolean setGyroFullScaleRange(GyroFullScaleRange fsr) throws RuntimeIOException {
        if (sensors == 0) {
            return false;
        }

        if (gyro_fsr == fsr) {
            return true;
        }

        Logger.info("Setting gyro config to 0x%x%n", fsr.getBitVal());
        // gyro_cfg == 0x1B == MPU9150_RA_GYRO_CONFIG
        i2cDevice.writeByteData(GYRO_CONFIG, fsr.getBitVal());
        gyro_fsr = fsr;

        return true;
    }

    /**
     * Set the accel full-scale range.
     *
     * @param fsr Desired full-scale range.
     * @throws RuntimeIOException if an I/O error occurs
     * @return status
     */
    public boolean mpu_set_accel_fsr(AccelFullScaleRange fsr) throws RuntimeIOException {
        if (sensors == 0) {
            return false;
        }

        if (accel_fsr == fsr) {
            return true;
        }

        System.out.format("Setting accel config to 0x%x%n", Byte.valueOf(fsr.getBitVal()));
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
     * @throws RuntimeIOException if an I/O error occurs
     * @return status
     */
    public boolean setSampleRate(int rate) throws RuntimeIOException {
        Logger.debug("mpu_set_sample_rate({})", rate);
        if (sensors == 0) {
            return false;
        }

        if (dmp_on) {
            Logger.warn("mpu_set_sample_rate() DMP is on, returning");
            return false;
        }

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
        mpuSetCompassSampleRate(Math.min(compass_sample_rate, MAX_COMPASS_SAMPLE_RATE));

        Logger.debug("Automatically setting LPF to {}", sample_rate >> 1);
        /* Automatically set LPF to 1/2 sampling rate. */
//        mpu_set_lpf(sample_rate >> 1);  TODO:

        return true;
    }

    public int mpu_get_compass_sample_rate() {
        return compass_sample_rate;
    }

    /**
     * Set compass sampling rate. The compass on the auxiliary I2C bus is read by the MPU
     * hardware at a maximum of 100Hz. The actual rate can be set to a fraction of the gyro
     * sampling rate.
     *
     * WARNING: The new rate may be different than what was requested. Call
     * mpu_get_compass_sample_rate to check the actual setting.
     *
     * @param rate Desired compass sampling rate (Hz).
     * @throws RuntimeIOException if an I/O error occurs
     * @return status
     */
    public boolean mpuSetCompassSampleRate(int rate) throws RuntimeIOException {
        Logger.debug("mpu_set_compass_sample_rate({})", rate);
        if (rate == 0 || rate > sample_rate || rate > MAX_COMPASS_SAMPLE_RATE) {
            return false;
        }

        byte div = (byte) (sample_rate / rate - 1);
        // s4_ctrl == 0x34 == MPU9150_RA_I2C_SLV4_CTRL
        i2cDevice.writeByteData(I2C_SLV4_CTRL, div);
        compass_sample_rate = sample_rate / (div + 1);

        return true;
    }

    /**
     * Get gyro sensitivity scale factor.
     *
     * @return Sensitivy, conversion from hardware units to dps.
     */
    public double getGyroSensitivity() {
        /*
         * float sens; switch (gyro_fsr) { case INV_FSR_250DPS: sens = 131.0f; break; case
         * INV_FSR_500DPS: sens = 65.5f; break; case INV_FSR_1000DPS: sens = 32.8f; break; case
         * INV_FSR_2000DPS: sens = 16.4f; break; default: sens = -1f; }
         *
         * return sens;
         */
        return gyro_fsr.getSensitivityScaleFactor();
    }

    /**
     * Get accel sensitivity scale factor.
     *
     * @return Sensitivity. Conversion from hardware units to g's.
     */
    public int mpu_get_accel_sens() {
        /*
         * int sens; switch (accel_fsr) { case INV_FSR_2G: sens = 16384; break; case INV_FSR_4G:
         * sens = 8192; break; case INV_FSR_8G: sens = 4096; break; case INV_FSR_16G: sens = 2048;
         * break; default: return -1; }
         */
        return accel_fsr.getSensitivityScaleFactor();
    }

    /**
     * Get current FIFO configuration. sensors can contain a combination of the following
     * flags: INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO INV_XYZ_GYRO INV_XYZ_ACCEL
     *
     * @return sensors Mask of sensors in FIFO.
     */
    public byte mpu_get_fifo_config() {
        return fifo_enable;
    }

    /**
     * Get current power state.
     *
     * @return power_on 1 if turned on, 0 if suspended.
     */
    public boolean mpu_get_power_state() {
        if (sensors != 0) {
            return true;
        }
        return false;
    }

    /**
     * Set interrupt level.
     *
     * @param active_low 1 for active low, 0 for active high.
     */
    public void mpu_set_int_level(boolean active_low) {
        active_low_int = active_low;
    }

    public float[] get_accel_prod_shift() throws RuntimeIOException {
        byte[] tmp = new byte[4];
        i2cDevice.readI2CBlockData(0x0D, tmp);
        // if (i2c_read(st.hw->addr, 0x0D, 4, tmp))
        // return 0x07;

        float[] st_shift = new float[3];
        byte[] shift_code = new byte[3];
        shift_code[0] = (byte) (((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4));
        shift_code[1] = (byte) (((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2));
        shift_code[2] = (byte) (((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03));
        for (int ii = 0; ii < 3; ii++) {
            if (shift_code[ii] == 0) {
                st_shift[ii] = 0.f;
                continue;
            }
            /*
             * Equivalent to.. st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
             */
            st_shift[ii] = 0.34f;
            while (--shift_code[ii] != 0) {
                st_shift[ii] *= 1.034f;
            }
        }
        return st_shift;
    }

    public void get_st_biases() throws RuntimeIOException {
        // TODO Implementation
        Logger.error("get_st_biases NOT IMPLEMENTED!");
    }


    /**
     * Get DMP state.
     *
     * @return enabled true if enabled.
     */
    public boolean mpu_get_dmp_state() {
        return dmp_on;
    }

    /* This initialisation is similar to the one in ak8975.c. */
    public boolean setupCompass() throws RuntimeIOException {

        Logger.info("Configure Magnetometer AK8963");

        // First extract the factory calibration for each magnetometer axis
        i2cDevice.writeByteData(AK8963_CNTL1,(byte) 0x00); // #KW 836 Power down magnetometer
        SleepUtil.sleepMillis(10);

        i2cDevice.writeByteData(AK8963_CNTL1, (byte)0x0F); // #KW 838 Enter Fuse ROM access bits
        SleepUtil.sleepMillis(10);

        byte calibrationValues[] = i2cDevice.readI2CBlockDataByteArray(AK8963_ASAX, 3);  // Read the x-, y-, and z-axis calibration values
        this.magCalibration = new float[]{
                ((float) (calibrationValues[0] - 128)) / 256f + 1f,   // #KW 841-843 Return x-axis sensitivity adjustment values, etc.
                ((float) (calibrationValues[1] - 128)) / 256f + 1f,
                ((float) (calibrationValues[2] - 128)) / 256f + 1f
        };

        i2cDevice.writeByteData(AK8963_CNTL1, (byte) 0x00); // #KW 844 Power down magnetometer
        SleepUtil.sleepMillis(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL1 register,
        // and enable continuous bits data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        // set to MagScale.MFS_16BIT.bits and MagMode.MM_100HZ set as final lines 48 & 49. register write should be 0x16

        i2cDevice.writeByte(AK8963_CNTL1, (byte)(magScale.bits | magMode.bits)); // #KW 849 Set magnetometer data resolution and sample ODR ####16bit already shifted
        SleepUtil.sleepMillis(10);
    }

    /**
     * Read raw compass data.
     *
     * NOTE: Uncalibrated
     *
     * @return data Raw data in hardware units.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public short[] mpu_get_compass_reg() throws RuntimeIOException {
        byte dataReady = (byte) (i2cDevice.readByteData(AK8963_ST1) & 0x01); //DRDY - Data ready bit0 1 = data is ready
        if (dataReady == 0) throw new RuntimeIOException("Not ready");

        short magX, magY, magZ;

        // #KW 494 readMagData - data is ready, read it NB bug fix here read was starting from ST1 not XOUT_L
        byte[] buffer = i2cDevice.readI2CBlockDataByteArray(AK8963_XOUT_L, 7); // #KW L815 6 data bytes x,y,z 16 bits stored as little Endian (L/H)
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

//            lastCalibratedReading = scale(new TimestampedData3f(	lastRawMagX*magScale.res*magCalibration.getX() - getDeviceBias().getX(),
//                    lastRawMagY*magScale.res*magCalibration.getY() - getDeviceBias().getY(),
//                    lastRawMagZ*magScale.res*magCalibration.getZ() - getDeviceBias().getZ()));
//            this.addValue(lastCalibratedReading); //store the result

            return new short[]{magX, magY, magZ};
        }

        return new short[]{0, 0, 0}; //TODO: check if it's correct behaviour
    }

    /**
     * Get the compass measurement range.
     *
     * @return fsr Current full-scale range.
     */
    public int getCompassFSR() {
        return AK8963_M_RANGE;
    }

    /**
     * Enters LP accel motion interrupt mode. The behaviour of this feature is very different
     * between the MPU6050 and the MPU6500. Each chip's version of this feature is explained
     * below.
     *
     * The hardware motion threshold can be between 32mg and 8160mg in 32mg increments.
     *
     * Low-power accel mode supports the following frequencies: 1.25Hz, 5Hz, 20Hz, 40Hz
     *
     * MPU6500: Unlike the MPU6050 version, the hardware does not "lock in" a reference
     * sample. The hardware monitors the accel data and detects any large change over a short
     * period of time.
     *
     * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
     *
     * MPU6500 Low-power accel mode supports the following frequencies: 1.25Hz, 2.5Hz, 5Hz,
     * 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
     *
     * NOTES: The driver will round down thresh to the nearest supported value if an
     * unsupported threshold is selected. To select a fractional wake-up frequency, round down
     * the value passed to lpa_freq. The MPU6500 does not support a delay parameter. If this
     * function is used for the MPU6500, the value passed to time will be ignored. To disable
     * this mode, set lpa_freq to zero. The driver will restore the previous configuration.
     *
     * @param thresh   Motion threshold in mg.
     * @param time     Duration in milliseconds that the accel data must exceed thresh before
     *                 motion is reported.
     * @param lpa_freq Minimum sampling rate, or zero to disable.
     * @throws RuntimeIOException if an I/O error occurs
     */
    public void mpu_lp_motion_interrupt(int thresh, int time, int lpa_freq) throws RuntimeIOException {
        // TODO Implementation
        Logger.error("mpu_lp_motion_interrupt NOT IMPLEMENTED!");
    }
}
