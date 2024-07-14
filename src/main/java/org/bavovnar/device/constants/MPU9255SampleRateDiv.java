package org.bavovnar.registers;

public enum SampleRateDiv {
    NONE((byte) 0, 1000),  //rates given are for a 1KHz base rate, for other base frequencies calculate accordingly
    HZ200((byte) 4, 200),
    HZ100((byte) 9, 100),
    HZ050((byte) 19, 50),
    HZ010((byte) 99, 10),
    HZ001((byte) 999, 1);

    final byte bits;
    final int rate1KHz;
    final static byte bitMask = (byte) 0xFF;

    SampleRateDiv(byte bits, int rate) {
        this.bits = bits;
        this.rate1KHz = rate;
    }

    public byte getBits() {
        return bits;
    }
}
