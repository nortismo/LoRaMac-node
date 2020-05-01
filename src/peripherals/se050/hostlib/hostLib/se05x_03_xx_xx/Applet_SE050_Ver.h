/* Copyright 2019 NXP
 * 
 * This software is owned or controlled by NXP and may only be used
 * strictly in accordance with the applicable license terms.  By expressly
 * accepting such terms or by downloading, installing, activating and/or
 * otherwise using the software, you are agreeing that you have read, and
 * that you agree to comply with and are bound by, such license terms.  If
 * you do not agree to be bound by the applicable license terms, then you
 * may not retain, install, activate or otherwise use the software.
 * 
 */

#ifndef APPLET_SE050_VERSION_INFO_H_INCLUDED
#define APPLET_SE050_VERSION_INFO_H_INCLUDED


/* clang-format off */
#define APPLET_SE050_PROD_NAME          "Applet_SE050"
#define APPLET_SE050_VER_STRING_NUM     "v03.01.00"
#define APPLET_SE050_PROD_NAME_VER_FULL "Applet_SE050_v03.01.00"
#define APPLET_SE050_VER_MAJOR          (3u)
#define APPLET_SE050_VER_MINOR          (1u)
#define APPLET_SE050_VER_DEV            (0u)

/* v03.01 = 30001u */
#define APPLET_SE050_VER_MAJOR_MINOR ( 0 \
    | (APPLET_SE050_VER_MAJOR * 10000u)    \
    | (APPLET_SE050_VER_MINOR))

/* v03.01.00 = 300010000ULL */
#define APPLET_SE050_VER_MAJOR_MINOR_DEV ( 0 \
    | (APPLET_SE050_VER_MAJOR * 10000*10000u)    \
    | (APPLET_SE050_VER_MINOR * 10000u)    \
    | (APPLET_SE050_VER_DEV))

/* clang-format on */


/* Version Information:
 * Generated by:
 *     ..\..\..\scripts\version_info.py (v2019.01.17_00)
 * 
 * Do not edit this file. Update:
 *     ./version_info.txt instead.
 * 
 * 
 * prod_name = "Applet_SE050"
 * 
 * prod_desc = "Applet AR6"
 * 
 * lang_c_prefix = prod_name.upper()
 * 
 * lang_namespace = ""
 * 
 * v_major  = "03"
 * 
 * v_minor  = "01"
 * 
 * v_dev    = "00"
 * 
 * v_meta   = ""
 * 
 * maturity = "P"
 * 
 */

#endif /* APPLET_SE050_VERSION_INFO_H_INCLUDED */
