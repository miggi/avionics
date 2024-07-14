package org.bavovnar.device.constants;

public enum MPU9255DLPBandwidth {
    //Configuration register 1A 26 bits 2:0  #### contains gyro and Thermometer DLFP bits ####
    //The DLPF is configured by DLPF_CFG, when FCHOICE_B [1:0] = 2b’00. The gyroscope and
    //temperature sensor are filtered according to the bits of DLPF_CFG as shown in
    //the table below, and FCHOICE_B stored in Gyroscope Configuration register 1B 27 bits 1:0
    //Note that FCHOICE mentioned in the table below is the inverted bits of FCHOICE_B
    //(e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).

    F32BW8800((byte)0,8800, 0.064f, 32, 4000, 0.04f), //DLPF bits not relevant (bits)
    F32BW3600((byte)0,3600, 0.11f, 32, 4000, 0.04f),
    F08BW0250((byte)0,250, 0.97f, 8, 4000, 0.04f),
    F01BW0184((byte)1,184, 2.9f, 1, 188, 1.9f),
    F01BW0092((byte)2,92, 3.9f, 1, 98, 2.8f),
    F01BW0041((byte)3,41, 5.9f, 1, 42, 4.8f),
    F01BW0020((byte)4,20, 9.9f, 1, 20, 8.3f),
    F01BW0010((byte)5,10, 17.85f, 1, 10, 13.4f),
    F01BW0005((byte)6,5, 33.48f, 1, 5, 18.6f),
    F08BW3600((byte)7,3600, 0.17f, 8, 4000, 0.04f);

    public final byte bits;
    public final int gyroBandWidth;
    public final float gyroDelay;
    public final int gyroRateKHz;
    public final int thermBandwidth;
    public final float thermDelay;

    public final static byte bitMask = (byte) 0x07;

    MPU9255DLPBandwidth(byte b, int gbw, float gd,int gf, int tbw, float tf)
    {
        bits = b;
        gyroBandWidth = gbw;
        gyroDelay = gd;
        gyroRateKHz = gf;
        thermBandwidth =tbw;
        thermDelay = tf;
    }
}
