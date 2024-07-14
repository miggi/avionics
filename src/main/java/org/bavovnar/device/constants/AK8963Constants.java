package org.bavovnar.device.constants;

/*
 * #%L
 * Organisation: diozero
 * Project:      diozero - IMU device classes
 * Filename:     AK8975Constants.java
 * 
 * This file is part of the diozero project. More information about this project
 * can be found at https://www.diozero.com/.
 * %%
 * Copyright (C) 2016 - 2024 diozero
 * %%
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * #L%
 */

public interface AK8963Constants {

	public static final int AK8963_ADDRESS   = 0x0C; // i2c bus address
	public static final int WHO_AM_I_AK8963 = 0x00; // should return  = 0x48
	public static final int INFO = 0x01;

	public static final int AK8963_WHO_AM_I  = 0x00; // should return (0x48
	public static final int AK8963_INFO      = 0x01;
	public static final int AK8963_ST1       = 0x02;  // data ready status bit 0
	public static final int AK8963_XOUT_L    = 0x03;  // data
	public static final int AK8963_XOUT_H    = 0x04;
	public static final int AK8963_YOUT_L    = 0x05;
	public static final int AK8963_YOUT_H    = 0x06;
	public static final int AK8963_ZOUT_L    = 0x07;
	public static final int AK8963_ZOUT_H    = 0x08;
	public static final int AK8963_ST2       = 0x09;  // Data overflow bit 3 and data read error status bit 2
	public static final int AK8963_CNTL1     = 0x0A;  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
	public static final int AK8963_CNTL2     = 0x0B;  // Reset bit 0
	public static final int AK8963_ASTC      = 0x0C;  // Self test control
	public static final int AK8963_I2CDIS    = 0x0F;  // device disable
	public static final int AK8963_ASAX      = 0x10;  // Fuse ROM x-axis sensitivity adjustment address
	public static final int AK8963_ASAY      = 0x11;  // Fuse ROM y-axis sensitivity adjustment address
	public static final int AK8963_ASAZ      = 0x12;  // Fuse ROM z-axis sensitivity adjustment address

	public static final int  AK8963_M_RANGE  = 4900;  // Fuse ROM z-axis sensitivity adjustment address

	static final int MAX_COMPASS_SAMPLE_RATE	= 100; // 100 Hz
}
