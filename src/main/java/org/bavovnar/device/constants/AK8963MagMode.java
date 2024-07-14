package org.bavovnar.device.constants;

public enum AK8963MagMode
{
    MM_100HZ   ((byte)0x06,1500), // 6 for 100 Hz ODR continuous magnetometer data read,new mag data is available every 10 ms
    MM_8HZ	 ((byte)0x02,128); // 2 for 8 Hz ODR, continuous magnetometer data read, new mag data is available every 125 ms

    public final byte bits;
    public final int sampleCount;
    final static byte bitmask = (byte) 0x06;

    AK8963MagMode(byte mode,int sampleCount)
    {
        this.bits = mode;
        this.sampleCount = sampleCount;
    }
}
