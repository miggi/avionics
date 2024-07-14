package org.bavovnar.core;

import com.diozero.devices.imu.ImuData;
import org.hipparchus.complex.Quaternion;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

public class Imu extends ImuData {
    private double altitude;

    public Imu(Vector3D gyro, Vector3D accel, Quaternion quaternion, Vector3D compass, float temperature, double altitude, long timestamp) {
        super(gyro, accel, quaternion, compass, temperature, timestamp);
        this.altitude = altitude;
    }
    public double getAltitude() {
        return altitude;
    }

    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }
}
