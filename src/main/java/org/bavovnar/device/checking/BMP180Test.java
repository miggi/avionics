package org.bavovnar.device.checking;

import com.diozero.api.RuntimeIOException;
import com.diozero.devices.BMP180;
import com.diozero.util.SleepUtil;
import org.tinylog.Logger;

public class BMP180Test {
    private static final int ITERATIONS = 200;

    public static void main(String[] args) {
        int controller = 1;
        if (args.length > 0) {
            controller = Integer.parseInt(args[0]);
        }

        try (BMP180 bmp180 = new BMP180(controller, BMP180.Mode.STANDARD)) {
            bmp180.readCalibrationData();
            Logger.debug("Opened device");

            for (int i = 0; i < ITERATIONS; i++) {
                Logger.info("Temperature: {0.##} C. Pressure: {0.##} hPa", bmp180.getTemperatureCelsius(), bmp180.getPressure());
                SleepUtil.sleepSeconds(0.5);
            }
        } catch (RuntimeIOException ioe) {
            Logger.error(ioe, "Error: {}", ioe);
        }
    }
}
