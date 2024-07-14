package org.bavovnar.device.checking;

import com.diozero.api.ServoDevice;
import com.diozero.api.ServoTrim;
import com.diozero.devices.PCA9685;
import com.diozero.util.SleepUtil;
import org.tinylog.Logger;

public class PCA9685ServoTest {
    private static final long LARGE_DELAY = 500;
    private static final long SHORT_DELAY = 10;

    public static void main(String[] args) {
        if (args.length < 2) {
            Logger.error("Usage: {} <pwm frequency> <gpio>", PCA9685ServoTest.class.getName());
            System.exit(1);
        }
        int pwm_freq = Integer.parseInt(args[0]);
        int pin_number = Integer.parseInt(args[1]);
        test(pwm_freq, pin_number);
    }

    public static void test(int pwmFrequency, int gpio) {
        ServoTrim trim = ServoTrim.MG996R;
        try (PCA9685 pca9685 = new PCA9685(0, pwmFrequency);
            ServoDevice servo = ServoDevice.Builder.builder(gpio).setDeviceFactory(pca9685).setTrim(trim).build()) {
            Logger.info("Mid");
            pca9685.setDutyUs(gpio, trim.getMidPulseWidthUs());
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Max");
            pca9685.setDutyUs(gpio, trim.getMaxPulseWidthUs());
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Mid");
            pca9685.setDutyUs(gpio, trim.getMidPulseWidthUs());
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Min");
            pca9685.setDutyUs(gpio, trim.getMinPulseWidthUs());
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Mid");
            pca9685.setDutyUs(gpio, trim.getMidPulseWidthUs());
            SleepUtil.sleepMillis(LARGE_DELAY);

            Logger.info("Max");
            servo.max();
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Mid");
            servo.mid();
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Min");
            servo.min();
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("Mid");
            servo.mid();
            SleepUtil.sleepMillis(LARGE_DELAY);

            Logger.info("0");
            servo.setAngle(0);
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("90 (Centre)");
            servo.setAngle(90);
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("180");
            servo.setAngle(180);
            SleepUtil.sleepMillis(LARGE_DELAY);
            Logger.info("90 (Centre)");
            servo.setAngle(90);
            SleepUtil.sleepMillis(LARGE_DELAY);

            for (int pulse_us = trim.getMidPulseWidthUs(); pulse_us < trim.getMaxPulseWidthUs(); pulse_us += 10) {
                servo.setPulseWidthUs(pulse_us);
                SleepUtil.sleepMillis(SHORT_DELAY);
            }
            for (int pulse_us = trim.getMaxPulseWidthUs(); pulse_us > trim.getMinPulseWidthUs(); pulse_us -= 10) {
                servo.setPulseWidthUs(pulse_us);
                SleepUtil.sleepMillis(SHORT_DELAY);
            }
            for (int pulse_us = trim.getMinPulseWidthUs(); pulse_us < trim.getMidPulseWidthUs(); pulse_us += 10) {
                servo.setPulseWidthUs(pulse_us);
                SleepUtil.sleepMillis(SHORT_DELAY);
            }
        }
    }
}
