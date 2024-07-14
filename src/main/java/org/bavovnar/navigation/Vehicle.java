package org.bavovnar.navigation;

import org.bavovnar.core.legacy.TimestampedData3f;

import java.time.Instant;
import java.time.ZoneId;
import java.time.ZonedDateTime;

public class Vehicle
{
    private static final String REMOTE_NAME = "Instruments";
    //Time of last instrument update
    private Instant updatedTimestamp;

    //data from individual sensors
    private TimestampedData3f magnetometer;
    private  TimestampedData3f accelerometer;
    private  TimestampedData3f gyroscope;

    //Fused  data from several sensors
    private  Quaternion quaternion;
    private  TimestampedData3f taitBryanAnglesR; //in radians yaw not adjusted 360 or for location declination
    private  TimestampedData3f taitBryanAnglesD; //in degrees yaw adjusted 360 and for location declination
    private  TimestampedData3f eulerAnglesR;
    private  TimestampedData3f eulerAnglesD;

    //in degrees adjusted for location and yaw to read 0-360
    private float yaw; 		//Yaw is the angle between SensorPackage x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    private float pitch; 	//Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    private float roll; 	//Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    //Alternate names for the same things are heading, attitude and bank
    @SuppressWarnings("CanBeFinal")
    private TimestampedData3f linearAcceleration;

    Vehicle()
    {
        updatedTimestamp = Instant.now();
        magnetometer = new TimestampedData3f(0,0,0);
        accelerometer = new TimestampedData3f(0,0,0);
        gyroscope = new TimestampedData3f(0,0,0);
        quaternion = new Quaternion();
        taitBryanAnglesR = new TimestampedData3f(0,0,0);
        taitBryanAnglesD = new TimestampedData3f(0,0,0);
        eulerAnglesR  = new TimestampedData3f(0,0,0);
        eulerAnglesD  = new TimestampedData3f(0,0,0);

        yaw = 0;
        pitch = 0;
        roll = 0;

        linearAcceleration = new TimestampedData3f(0,0,0);
    }

    // getters
    public Instant getTimestamp(){return updatedTimestamp;}
    public ZonedDateTime getDateTime() {return updatedTimestamp.atZone(ZoneId.of("GB"));}
    public float getYaw() {return yaw;}
    public float getPitch() {return pitch;}
    public float getRoll() {return roll;}
    public float getHeading() {return getYaw();}
    public float getAttitude() {return getPitch();}
    public float getBank() {return getRoll();}
    public Instant getUpdatedTimestamp() {return updatedTimestamp;}
    public Quaternion getQuaternion() {return quaternion;}
    public TimestampedData3f getTaitBryanAnglesR() {return taitBryanAnglesR;}
    public TimestampedData3f getTaitBryanAnglesD() {return new TimestampedData3f(taitBryanAnglesD);}
    public TimestampedData3f getMagnetometer() {return magnetometer;}
    public TimestampedData3f getAccelerometer() {return accelerometer;}
    public TimestampedData3f getGyroscope() {return gyroscope;}
    public TimestampedData3f getAngles(){return new TimestampedData3f(yaw,pitch,roll);}
    public TimestampedData3f getLinearAcceleration() {return linearAcceleration;}
    public TimestampedData3f getEulerAnglesR()	{return eulerAnglesR;}
    public TimestampedData3f getEulerAnglesD()	{return eulerAnglesD;}

    public static String getRemoteName() {return REMOTE_NAME;}
    //Setters
    void setMagnetometer(TimestampedData3f magnetometer) {this.magnetometer = magnetometer;}
    void setAccelerometer(TimestampedData3f accelerometer) {this.accelerometer = accelerometer;}
    void setGyroscope(TimestampedData3f gyroscope) {this.gyroscope = gyroscope;}

    //public void setYaw(float yaw) {Instruments.yaw = yaw;}

    //public void setHeading(float heading) {Instruments.yaw = heading;}

    //public void setAttitude(float attitude) {Instruments.pitch = attitude;}

    //public void setPitch(float pitch) {Instruments.pitch = pitch;}

    //public void setBank(float bank) {Instruments.roll = bank;}

    //public void setRoll(float roll) {Instruments.roll = roll;}

    /**
     * Update output acceleration variables Yaw, Pitch and Roll based on fused sensor data
     * <p>
     * these are Tait-Bryan angles, commonly used in aircraft orientation.
     * In this coordinate system, the positive z-axis is down toward Earth.
     * Yaw is the angle between SensorPackage x-axis and Earth magnetic North (or true North if corrected for local declination,
     * looking down on the sensor positive yaw is counterclockwise.
     * Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
     * Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
     * <p>
     * These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
     * Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
     * applied in the correct order which for this configuration is yaw, pitch, and then roll.
     * For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
     *
     * @param q a quaternion containing the fused input data - see https://en.wikipedia.org/wiki/Quaternion
     */
    void updateVehicle(Quaternion q)
    {
        if (q == null) return; // don't do anything

        updatedTimestamp = Instant.now();
        quaternion = q;
        taitBryanAnglesR = q.toTaitBryanAngles();
        eulerAnglesR = q.toEulerianAngles();
        eulerAnglesD = new TimestampedData3f(	(float) Math.toDegrees(eulerAnglesR.getX()),
                (float) Math.toDegrees(eulerAnglesR.getY()),
                (float) Math.toDegrees(eulerAnglesR.getZ()));

        yaw   = (float) Math.toDegrees(taitBryanAnglesR.getX()); 	//radians to degrees		// #KW L634
        pitch = (float) Math.toDegrees(taitBryanAnglesR.getY()); 	//radians to degrees		// #KW L633
        roll  = (float) Math.toDegrees(taitBryanAnglesR.getZ()); 	//radians to degrees		// #KW L637

        // #KW L635 yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
        yaw += -44.0f / 60.0f; // Declination at Letchworth England is minus O degrees and 44 Seconds on 2016-07-11
        if (yaw < 0) yaw += 360.0f; // Ensure heading stays between 0 and 360
        taitBryanAnglesD = new TimestampedData3f(yaw, pitch, roll);
        updateLinearAcceleration(quaternion);
    }
    private void updateLinearAcceleration(Quaternion q) {
        float a31 = 2.0f * (q.w * q.x + q.y * q.z);
        float a32 = 2.0f * (q.x * q.z - q.w * q.y);
        float a33 = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
        linearAcceleration.setX(accelerometer.getX() + a31);
        linearAcceleration.setY(accelerometer.getY() + a32);
        linearAcceleration.setZ(accelerometer.getZ() - a33);
    }
}
