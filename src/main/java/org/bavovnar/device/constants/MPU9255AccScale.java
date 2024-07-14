package org.bavovnar.device.constants;

public enum MPU9255AccScale { //Register 28 – Accelerometer Configuration bits 4:3
    AFS_2G((byte) 0x00, 2),
    AFS_4G((byte) 0x08, 4),
    AFS_8G((byte) 0x10, 8),
    AFS_16G((byte) 0x18, 16);
    public final byte bits;
    public final int minMax;
    final static byte bitMask = (byte) 0x18;

    MPU9255AccScale(byte value, int minMax) {
        this.bits = value;
        this.minMax = minMax;
    }

    public float getRes() {
        return ((float) minMax) / 32768.0f;
    }
}
