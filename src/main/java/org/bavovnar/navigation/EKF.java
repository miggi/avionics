package org.bavovnar.navigation;

import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;

public class EKF {

    public static double[] gpsToEcef(double latitude, double longitude, double altitude) {
        double latRad = Math.toRadians(latitude);
        double lonRad = Math.toRadians(longitude);
        double a = 6378137.0; // Earth radius in meters
        double e2 = 0.00669437999014; // Square of eccentricity
        double N = a / Math.sqrt(1 - e2 * Math.sin(latRad) * Math.sin(latRad));

        double x = (N + altitude) * Math.cos(latRad) * Math.cos(lonRad);
        double y = (N + altitude) * Math.cos(latRad) * Math.sin(lonRad);
        double z = ((1 - e2) * N + altitude) * Math.sin(latRad);

        return new double[]{x, y, z};
    }

    RealVector initialState = new ArrayRealVector(new double[] {1, 0, 0, 0, 0, 0, 0, 0, 0, 0}); // Quaternion (1,0,0,0) and zero for position and velocity
    RealMatrix initialCovariance = MatrixUtils.createRealIdentityMatrix(10); // Simplified initial covariance

// The state transition function predicts the next state based on the current state and the control input. For orientation,
// this involves integrating the angular velocity (from the gyroscope) to update the quaternion. For position and velocity, simple kinematic equations are used.

   /*
    public RealMatrix getStateTransitionMatrix(RealVector state, double dt) {
        // Extract the quaternion and velocity
        double[] q = Arrays.copyOfRange(state.toArray(), 0, 4);
        double[] v = Arrays.copyOfRange(state.toArray(), 7, 10);

        // Quaternion derivative from angular velocity (gyro measurements)
        double[] omega = {gyroX, gyroY, gyroZ}; // Angular velocities from the gyroscope
        Quaternion omegaQuat = new Quaternion(0, omega);
        Quaternion stateQuat = new Quaternion(q);
        Quaternion quatDerivative = stateQuat.multiply(omegaQuat).scalarMultiply(0.5);

        // Update quaternion by integrating the derivative
        Quaternion newQuat = stateQuat.add(quatDerivative.scalarMultiply(dt)).normalize();

        // Predict new position and velocity using simple kinematics
        double[] newPos = new double[3];
        double[] newVel = new double[3];
        for (int i = 0; i < 3; i++) {
            newVel[i] = v[i] + accel[i] * dt; // Acceleration from accelerometer
            newPos[i] = state.getEntry(i + 4) + v[i] * dt + 0.5 * accel[i] * dt * dt;
        }

        // Construct the new state vector
        RealVector newState = new ArrayRealVector(10);
        newState.setSubVector(0, new ArrayRealVector(newQuat.getQ()));
        newState.setSubVector(4, new ArrayRealVector(newPos));
        newState.setSubVector(7, new ArrayRealVector(newVel));
        return newState;
    }




    // Extract orientation as a quaternion
    Quaternion orientation = new Quaternion(
            ekf.getStateEstimation().getEntry(0),
            ekf.getStateEstimation().getEntry(1),
            ekf.getStateEstimation().getEntry(2),
            ekf.getStateEstimation().getEntry(3)
    );

    // Extract position
    double posX = ekf.getStateEstimation().getEntry(4);
    double posY = ekf.getStateEstimation().getEntry(5);
    double posZ = ekf.getStateEstimation().getEntry(6);

System.out.println("Orientation (Quaternion): " + orientation);
System.out.println("Position: (" + posX + ", " + posY + ", " + posZ + ")");

*/
}
