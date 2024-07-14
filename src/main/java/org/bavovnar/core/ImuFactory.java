package org.bavovnar.core;

import org.hipparchus.complex.Quaternion;
import org.hipparchus.geometry.euclidean.threed.Vector3D;

public class ImuFactory {

    static final int MPU9150_QUAT_W = 0;
    static final int MPU9150_QUAT_X = 1;
    static final int MPU9150_QUAT_Y = 2;
    static final int MPU9150_QUAT_Z = 3;

    public static Vector3D createVector(short[] data, double scale) {
        return new Vector3D(
                Utils.roundThreeDigits(data[0] * scale),
                Utils.roundThreeDigits(data[1] * scale),
                Utils.roundThreeDigits(data[2] * scale)
        );
    }

    public static Vector3D createVector(float[] data, double scale) {
        return new Vector3D(
                Utils.roundThreeDigits(data[0] * scale),
                Utils.roundThreeDigits(data[1] * scale),
                Utils.roundThreeDigits(data[2] * scale)
        );
    }

    public static Quaternion createQuaternion(int[] quat, double scale) {
        // This article suggests QUAT_W is [0]
        // https://github.com/vmayoral/bb_mpu9150/blob/master/src/linux-mpu9150/mpu9150/mpu9150.c
        Quaternion quaterion = new Quaternion(quat[MPU9150_QUAT_W] * scale, quat[MPU9150_QUAT_X] * scale,
                quat[MPU9150_QUAT_Y] * scale, quat[MPU9150_QUAT_Z] * scale);
        return quaterion.normalize();
    }

    public static Imu newInstance(float[] gyro,
                                  float[] accel,
                                  float[] compass,
                                  float temperature,
                                  double altitude,
                                  double gyroScale,
                                  double accelScale,
                                  double compassScale
    ) {

        return new Imu(
                createVector(gyro, gyroScale),
                createVector(accel, accelScale), null,
                createVector(compass, compassScale), temperature, altitude, System.currentTimeMillis());
    }

}
