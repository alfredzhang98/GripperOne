/**
 * @file wit_reg.h
 * @brief WitMotion WIT-standard register map + protocol constants (JY901P etc).
 *
 * Full transcription of the register table so any register is on hand when we
 * need it. Private to the imu_uart component (kept out of include/), so it does
 * not become part of the public API — the driver's public surface is imu_uart.h.
 *
 * Config writes use the 5-byte frame  0xFF 0xAA <ADDR> <dataL> <dataH>  after a
 * KEY unlock; multi-byte values are little-endian (dataL first).
 *
 * Access tags on each register:  RW = read/write (configurable),  R = read-only
 * (poll via a register read; the streamed output frame carries the same data).
 */
#ifndef WIT_REG_H
#define WIT_REG_H

/* =========================================================================
 * Output data frame (device -> host)
 *   [0]=0x55  [1]=type  [2..9]=four int16 LE  [10]=checksum(sum of [0..9] & 0xFF)
 * ========================================================================= */
#define IMU_FRAME_HEAD     0x55
#define IMU_FRAME_LEN      11

/* type byte (frame[1]) */
#define IMU_TYPE_TIME      0x50
#define IMU_TYPE_ACC       0x51
#define IMU_TYPE_GYRO      0x52
#define IMU_TYPE_ANGLE     0x53
#define IMU_TYPE_MAG       0x54
#define IMU_TYPE_PORT      0x55
#define IMU_TYPE_PRESS     0x56
#define IMU_TYPE_GPS       0x57
#define IMU_TYPE_VELOCITY  0x58
#define IMU_TYPE_QUAT      0x59
#define IMU_TYPE_GSA       0x5A
#define IMU_TYPE_READ      0x5F   /* register-read response */

/* =========================================================================
 * Register addresses (ADDR hex column).   RW = read/write,  R = read-only
 * ========================================================================= */
/* --- system / configuration (RW) --- */
#define IMU_REG_SAVE       0x00   /* RW  0x0000 save, 0x0001 factory reset, 0x00FF reboot */
#define IMU_REG_CALSW      0x01   /* RW  calibration mode, CALSW[3:0] */
#define IMU_REG_RSW        0x02   /* RW  output content bitmap */
#define IMU_REG_RRATE      0x03   /* RW  output rate */
#define IMU_REG_BAUD       0x04   /* RW  serial baud */

/* --- zero-bias offsets (RW) --- */
#define IMU_REG_AXOFFSET   0x05   /* RW */
#define IMU_REG_AYOFFSET   0x06   /* RW */
#define IMU_REG_AZOFFSET   0x07   /* RW */
#define IMU_REG_GXOFFSET   0x08   /* RW */
#define IMU_REG_GYOFFSET   0x09   /* RW */
#define IMU_REG_GZOFFSET   0x0A   /* RW */
#define IMU_REG_HXOFFSET   0x0B   /* RW */
#define IMU_REG_HYOFFSET   0x0C   /* RW */
#define IMU_REG_HZOFFSET   0x0D   /* RW */

/* --- IO / device --- */
#define IMU_REG_D0MODE     0x0E   /* RW */
#define IMU_REG_D1MODE     0x0F   /* RW */
#define IMU_REG_D2MODE     0x10   /* RW */
#define IMU_REG_D3MODE     0x11   /* RW */
#define IMU_REG_IICADDR    0x1A   /* RW */
#define IMU_REG_LEDOFF     0x1B   /* RW */
#define IMU_REG_MAGRANGX   0x1C   /* RW */
#define IMU_REG_MAGRANGY   0x1D   /* RW */
#define IMU_REG_MAGRANGZ   0x1E   /* RW */
#define IMU_REG_BANDWIDTH  0x1F   /* RW */
#define IMU_REG_GYRORANGE  0x20   /* RW */
#define IMU_REG_ACCRANGE   0x21   /* RW */
#define IMU_REG_SLEEP      0x22   /* RW */
#define IMU_REG_ORIENT     0x23   /* RW  install direction */
#define IMU_REG_AXIS6      0x24   /* RW  6/9-axis algorithm */
#define IMU_REG_FILTK      0x25   /* RW  dynamic filter */
#define IMU_REG_GPSBAUD    0x26   /* RW */
#define IMU_REG_READADDR   0x27   /* RW  register to read back */
#define IMU_REG_ACCFILT    0x2A   /* RW */
#define IMU_REG_POWONSEND  0x2D   /* RW */
#define IMU_REG_VERSION    0x2E   /* R   firmware version */

/* --- real-time data --- */
#define IMU_REG_YYMM       0x30   /* RW  year / month */
#define IMU_REG_DDHH       0x31   /* RW  day / hour */
#define IMU_REG_MMSS       0x32   /* RW  minute / second */
#define IMU_REG_MS         0x33   /* RW  millisecond */
#define IMU_REG_AX         0x34   /* R */
#define IMU_REG_AY         0x35   /* R */
#define IMU_REG_AZ         0x36   /* R */
#define IMU_REG_GX         0x37   /* R */
#define IMU_REG_GY         0x38   /* R */
#define IMU_REG_GZ         0x39   /* R */
#define IMU_REG_HX         0x3A   /* R */
#define IMU_REG_HY         0x3B   /* R */
#define IMU_REG_HZ         0x3C   /* R */
#define IMU_REG_ROLL       0x3D   /* R */
#define IMU_REG_PITCH      0x3E   /* R */
#define IMU_REG_YAW        0x3F   /* R */
#define IMU_REG_TEMP       0x40   /* R */
#define IMU_REG_D0STATUS   0x41   /* R */
#define IMU_REG_D1STATUS   0x42   /* R */
#define IMU_REG_D2STATUS   0x43   /* R */
#define IMU_REG_D3STATUS   0x44   /* R */
#define IMU_REG_PRESSUREL  0x45   /* R */
#define IMU_REG_PRESSUREH  0x46   /* R */
#define IMU_REG_HEIGHTL    0x47   /* R */
#define IMU_REG_HEIGHTH    0x48   /* R */
#define IMU_REG_LONL       0x49   /* R */
#define IMU_REG_LONH       0x4A   /* R */
#define IMU_REG_LATL       0x4B   /* R */
#define IMU_REG_LATH       0x4C   /* R */
#define IMU_REG_GPSHEIGHT  0x4D   /* R */
#define IMU_REG_GPSYAW     0x4E   /* R */
#define IMU_REG_GPSVL      0x4F   /* R */
#define IMU_REG_GPSVH      0x50   /* R */
#define IMU_REG_Q0         0x51   /* R */
#define IMU_REG_Q1         0x52   /* R */
#define IMU_REG_Q2         0x53   /* R */
#define IMU_REG_Q3         0x54   /* R */
#define IMU_REG_SVNUM      0x55   /* R */
#define IMU_REG_PDOP       0x56   /* R */
#define IMU_REG_HDOP       0x57   /* R */
#define IMU_REG_VDOP       0x58   /* R */

/* --- alarm / misc --- */
#define IMU_REG_DELAYT     0x59   /* RW  alarm signal delay */
#define IMU_REG_XMIN       0x5A   /* RW  X-angle alarm min */
#define IMU_REG_XMAX       0x5B   /* RW  X-angle alarm max */
#define IMU_REG_BATVAL     0x5C   /* R   supply voltage */
#define IMU_REG_ALARMPIN   0x5D   /* RW  alarm pin mapping */
#define IMU_REG_YMIN       0x5E   /* RW  Y-angle alarm min */
#define IMU_REG_YMAX       0x5F   /* RW  Y-angle alarm max */
#define IMU_REG_GYROCALITHR 0x61  /* RW  gyro still threshold (val/1000 deg/s); also
                                     gyro auto-cal switch: write 0=on(default), 1=off */
#define IMU_REG_ALARMLEVEL 0x62   /* RW  angle alarm level */
#define IMU_REG_GYROCALTIME 0x63  /* RW  gyro auto-cal time */
#define IMU_REG_TRIGTIME   0x68   /* RW  alarm continuous-trigger time */
#define IMU_REG_KEY        0x69   /* RW  unlock, value 0xB588 */
#define IMU_REG_WERROR     0x6A   /* R   gyro change value */
#define IMU_REG_TIMEZONE   0x6B   /* RW  GPS time zone */
#define IMU_REG_WZTIME     0x6E   /* RW  angular-velocity still time */
#define IMU_REG_WZSTATIC   0x6F   /* RW  angular-velocity integral threshold */
#define IMU_REG_MODDELAY   0x74   /* RW  485 response delay */
#define IMU_REG_XREFROLL   0x79   /* R   roll zero reference */
#define IMU_REG_YREFPITCH  0x7A   /* R   pitch zero reference */
#define IMU_REG_NUMBERID1  0x7F   /* R   device serial, id1..id2 */
#define IMU_REG_NUMBERID2  0x80   /* R */
#define IMU_REG_NUMBERID3  0x81   /* R */
#define IMU_REG_NUMBERID4  0x82   /* R */
#define IMU_REG_NUMBERID5  0x83   /* R */
#define IMU_REG_NUMBERID6  0x84   /* R */

/* =========================================================================
 * Field values
 * ========================================================================= */
/* SAVE (0x00) */
#define IMU_SAVE_CONFIG    0x0000
#define IMU_SAVE_FACTORY   0x0001
#define IMU_SAVE_REBOOT    0x00FF

/* CALSW (0x01) calibration modes */
#define IMU_CALSW_NORMAL   0x00   /* quit calibration */
#define IMU_CALSW_ACC      0x01   /* accelerometer */
#define IMU_CALSW_HEIGHT   0x03   /* height reset */
#define IMU_CALSW_YAWZERO  0x04   /* heading (yaw) -> 0 */
#define IMU_CALSW_MAG_SPH  0x07   /* magnetic, spherical fit */
#define IMU_CALSW_ANGLEREF 0x08   /* set angle reference (roll/pitch relative zero) */
#define IMU_CALSW_MAG_DUAL 0x09   /* magnetic, dual-plane */
#define IMU_CALSW_GYRO_BEG 0x0C   /* manual gyro cal: begin acquiring zero bias */
#define IMU_CALSW_GYRO_END 0x0D   /* manual gyro cal: end (auto-cal must be off) */

/* RSW (0x02) output-content bits */
#define IMU_RSW_TIME       (1u << 0)
#define IMU_RSW_ACC        (1u << 1)
#define IMU_RSW_GYRO       (1u << 2)
#define IMU_RSW_ANGLE      (1u << 3)
#define IMU_RSW_MAG        (1u << 4)
#define IMU_RSW_PORT       (1u << 5)
#define IMU_RSW_PRESS      (1u << 6)
#define IMU_RSW_GPS        (1u << 7)
#define IMU_RSW_VELOCITY   (1u << 8)
#define IMU_RSW_QUAT       (1u << 9)
#define IMU_RSW_GSA        (1u << 10)

/* RRATE (0x03) output-rate codes */
#define IMU_RRATE_0_2HZ    0x01
#define IMU_RRATE_0_5HZ    0x02
#define IMU_RRATE_1HZ      0x03
#define IMU_RRATE_2HZ      0x04
#define IMU_RRATE_5HZ      0x05
#define IMU_RRATE_10HZ     0x06
#define IMU_RRATE_20HZ     0x07
#define IMU_RRATE_50HZ     0x08
#define IMU_RRATE_100HZ    0x09
#define IMU_RRATE_200HZ    0x0B
#define IMU_RRATE_ONCE     0x0C
#define IMU_RRATE_NONE     0x0D

/* BAUD (0x04) codes */
#define IMU_BAUD_4800      0x01
#define IMU_BAUD_9600      0x02
#define IMU_BAUD_19200     0x03
#define IMU_BAUD_38400     0x04
#define IMU_BAUD_57600     0x05
#define IMU_BAUD_115200    0x06
#define IMU_BAUD_230400    0x07
#define IMU_BAUD_460800    0x08   /* WT931/JY931/HWT606/HWT906 only */
#define IMU_BAUD_921600    0x09   /* WT931/JY931/HWT606/HWT906 only */

/* KEY (0x69) unlock value */
#define IMU_KEY_UNLOCK     0xB588

#endif /* WIT_REG_H */
