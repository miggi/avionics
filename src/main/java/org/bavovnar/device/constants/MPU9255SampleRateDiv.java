package org.bavovnar.device.constants;

public enum MPU9255SampleRateDiv {
    NONE((byte) 0, 1000),  //rates given are for a 1KHz base rate, for other base frequencies calculate accordingly
    HZ200((byte) 4, 200),
    HZ100((byte) 9, 100),
    HZ050((byte) 19, 50),
    HZ010((byte) 99, 10),
    HZ001((byte) 999, 1);

    public final byte bits;
    public final int rate1KHz;
    public final static byte bitMask = (byte) 0xFF;

    MPU9255SampleRateDiv(byte bits, int rate) {
        this.bits = bits;
        this.rate1KHz = rate;
    }

    public byte getBits() {
        return bits;
    }
}
