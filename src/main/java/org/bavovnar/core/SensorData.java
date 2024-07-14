package org.bavovnar.core;

import org.bavovnar.core.legacy.Data3f;
import org.bavovnar.navigation.Quaternion;

import java.io.Serializable;

public class SensorData implements Serializable {
    private Float t;
    private Float alt;
    private Data3f gyro;
    private Data3f acc;
    private Data3f mag;
    private Quaternion q;

    public SensorData() {
        this.t = 0.0f;
        this.alt = 0.0f;

        this.gyro = new Data3f();
        this.acc = new Data3f();
        this.mag = new Data3f();
        this.q = new Quaternion();
    }

    public SensorData(Float temperature,
                      Float altitude,
                      Data3f g,
                      Data3f a,
                      Data3f m,
                      Quaternion q) {

        this.t = temperature;
        this.alt = altitude;

        this.gyro = g;
        this.acc = a;
        this.mag = m;
        this.q = q;
    }

    public Float getT() {
        return t;
    }

    public void setT(Float t) {
        this.t = t;
    }

    public Float getAlt() {
        return alt;
    }

    public void setAlt(Float alt) {
        this.alt = alt;
    }

    public Data3f getGyro() {
        return gyro;
    }

    public void setGyro(Data3f gyro) {
        this.gyro = gyro;
    }

    public Data3f getAcc() {
        return acc;
    }

    public void setAcc(Data3f acc) {
        this.acc = acc;
    }

    public Data3f getMag() {
        return mag;
    }

    public void setMag(Data3f mag) {
        this.mag = mag;
    }

    public Quaternion getQ() {
        return q;
    }

    public void setQ(Quaternion q) {
        this.q = q;
    }

    @Override
    public String toString() {
        return "SensorData{" +
                "t=" + t +
                ", alt=" + alt +
                ", gyro=" + gyro +
                ", acc=" + acc +
                ", mag=" + mag +
                ", q=" + q +
                '}';
    }
}
