package org.bavovnar.device.constants;

public enum AK8963MagScale { //#KW L722
    MFS_14BIT((byte) 0x00, 10f * 4912f / 8190f),  // #KW L728 mscale val = 0, 14 bit, 5.99755 #KW L234 comment says 0.6 mG per LSB
    MFS_16BIT((byte) 0x10, 10f * 4912f / 32760f); // #KW L731 mscale val = 1, 16 bit, 1.49939 #KW L235 comment says 0.15 mG per LSB

    public final byte bits;

    public final float res;

    public final byte bitMask = (byte) 0x10;
    AK8963MagScale(byte bits, float res)
    {
        this.bits = bits;
        this.res = res;
    }
}
