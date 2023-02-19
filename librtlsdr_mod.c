/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <libusb.h>

/*
 * All libusb callback functions should be marked with the LIBUSB_CALL macro
 * to ensure that they are compiled with the same calling convention as libusb.
 *
 * If the macro isn't available in older libusb versions, we simply define it.
 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#include "rtl-sdr.h"

enum qs1r_async_status {
    QS1R_INACTIVE = 0,
    QS1R_CANCELING,
    QS1R_RUNNING
};

struct rtlsdr_dev {
    libusb_context *ctx;
    struct libusb_device_handle *devh;
    uint32_t xfer_buf_num;
    uint32_t xfer_buf_len;
    struct libusb_transfer **xfer;
    unsigned char **xfer_buf;
    rtlsdr_read_async_cb_t cb;
    void *cb_ctx;
    enum qs1r_async_status async_status;
    int async_cancel;
    int use_zerocopy;
    /* rtl demod context */
    /* status */
    int dev_lost;
    int driver_active;
    unsigned int xfer_errors;
};

#define DEFAULT_BUF_NUMBER  15
#define DEFAULT_BUF_LENGTH  (16 * 32 * 512)

#define CTRL_IN     (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_TIMEOUT    300
#define BULK_TIMEOUT    0

#define EEPROM_ADDR 0xa0

enum usb_reg {
    USB_SYSCTL      = 0x2000,
    USB_CTRL        = 0x2010,
    USB_STAT        = 0x2014,
    USB_EPA_CFG     = 0x2144,
    USB_EPA_CTL     = 0x2148,
    USB_EPA_MAXPKT      = 0x2158,
    USB_EPA_MAXPKT_2    = 0x215a,
    USB_EPA_FIFO_CFG    = 0x2160,
};

enum blocks {
    DEMODB          = 0,
    USBB            = 1,
    SYSB            = 2,
    TUNB            = 3,
    ROMB            = 4,
    IRB         = 5,
    IICB            = 6,
};

int rtlsdr_read_array(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
    int r;
    uint16_t index = (block << 8);

    r = libusb_control_transfer(dev->devh, CTRL_IN, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);
#endif
    return r;
}

int rtlsdr_write_array(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
    int r;
    uint16_t index = (block << 8) | 0x10;

    r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);
#endif
    return r;
}

int rtlsdr_write_reg(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint16_t val, uint8_t len)
{
    int r;
    unsigned char data[2];

    uint16_t index = (block << 8) | 0x10;

    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;

    data[1] = val & 0xff;

    r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    return r;
}

uint16_t rtlsdr_demod_read_reg(rtlsdr_dev_t *dev, uint8_t page, uint16_t addr, uint8_t len)
{
    int r;
    unsigned char data[2];

    uint16_t index = page;
    uint16_t reg;
    addr = (addr << 8) | 0x20;

    r = libusb_control_transfer(dev->devh, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    reg = (data[1] << 8) | data[0];

    return reg;
}

int rtlsdr_demod_write_reg(rtlsdr_dev_t *dev, uint8_t page, uint16_t addr, uint16_t val, uint8_t len)
{
    int r;
    unsigned char data[2];
    uint16_t index = 0x10 | page;
    addr = (addr << 8) | 0x20;

    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;

    data[1] = val & 0xff;

    r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1);

    return (r == len) ? 0 : -1;
}

int rtlsdr_open(rtlsdr_dev_t **out_dev, uint32_t index)
{
    int r;
    int i;
    libusb_device **list;
    rtlsdr_dev_t *dev = NULL;
    libusb_device *device = NULL;
    uint32_t device_count = 0;
    struct libusb_device_descriptor dd;
    uint8_t reg;
    ssize_t cnt;



    r = libusb_open(device, &dev->devh);

    /* perform a dummy write, if it fails, reset the device */
    if (rtlsdr_write_reg(dev, USBB, USB_SYSCTL, 0x09, 1) < 0) {
        fprintf(stderr, "Resetting device...\n");
        libusb_reset_device(dev->devh);
    }

    //rtlsdr_init_baseband(dev);
    dev->dev_lost = 0;

    /* initialise GPIOs */

    /* reset tuner before probing */

found:

    return 0;
err:
    if (dev) {
        if (dev->devh)
            libusb_close(dev->devh);

        if (dev->ctx)
            libusb_exit(dev->ctx);

        free(dev);
    }

    return r;
}

int rtlsdr_close(rtlsdr_dev_t *dev)
{
    /* block until all async operations have been completed (if any) */
    while (QS1R_INACTIVE != dev->async_status) {
#ifdef _WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
    }

//    libusb_release_interface(dev->devh, 0);


  //  libusb_close(dev->devh);

    //libusb_exit(dev->ctx);

    //free(dev);

    return 0;
}

static void LIBUSB_CALL _libusb_callback(struct libusb_transfer *xfer)
{
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)xfer->user_data;

    if (LIBUSB_TRANSFER_COMPLETED == xfer->status) {
        if (dev->cb)
            dev->cb(xfer->buffer, xfer->actual_length, dev->cb_ctx);

        libusb_submit_transfer(xfer); /* resubmit transfer */
        dev->xfer_errors = 0;
    } else if (LIBUSB_TRANSFER_CANCELLED != xfer->status) {
#ifndef _WIN32
        if (LIBUSB_TRANSFER_ERROR == xfer->status)
            dev->xfer_errors++;

        if (dev->xfer_errors >= dev->xfer_buf_num ||
            LIBUSB_TRANSFER_NO_DEVICE == xfer->status) {
#endif
            dev->dev_lost = 1;
            rtlsdr_cancel_async(dev);
            fprintf(stderr, "cb transfer status: %d, "
                "canceling...\n", xfer->status);
#ifndef _WIN32
        }
#endif
    }
}

int rtlsdr_wait_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, void *ctx)
{
    return rtlsdr_read_async(dev, cb, ctx, 0, 0);
}

static int _rtlsdr_alloc_async_buffers(rtlsdr_dev_t *dev)
{
    unsigned int i;

    if (!dev)
        return -1;

    if (!dev->xfer) {
        dev->xfer = malloc(dev->xfer_buf_num *
                   sizeof(struct libusb_transfer *));

        for(i = 0; i < dev->xfer_buf_num; ++i)
            dev->xfer[i] = libusb_alloc_transfer(0);
    }

    if (dev->xfer_buf)
        return -2;

    dev->xfer_buf = malloc(dev->xfer_buf_num * sizeof(unsigned char *));
    memset(dev->xfer_buf, 0, dev->xfer_buf_num * sizeof(unsigned char *));

#if defined(ENABLE_ZEROCOPY) && defined (__linux__) && LIBUSB_API_VERSION >= 0x01000105
    fprintf(stderr, "Allocating %d zero-copy buffers\n", dev->xfer_buf_num);

    dev->use_zerocopy = 1;
    for (i = 0; i < dev->xfer_buf_num; ++i) {
        dev->xfer_buf[i] = libusb_dev_mem_alloc(dev->devh, dev->xfer_buf_len);

        if (dev->xfer_buf[i]) {
            /* Check if Kernel usbfs mmap() bug is present: if the
             * mapping is correct, the buffers point to memory that
             * was memset to 0 by the Kernel, otherwise, they point
             * to random memory. We check if the buffers are zeroed
             * and otherwise fall back to buffers in userspace.
             */
            if (dev->xfer_buf[i][0] || memcmp(dev->xfer_buf[i],
                              dev->xfer_buf[i] + 1,
                              dev->xfer_buf_len - 1)) {
                fprintf(stderr, "Detected Kernel usbfs mmap() "
                        "bug, falling back to buffers "
                        "in userspace\n");
                dev->use_zerocopy = 0;
                break;
            }
        } else {
            fprintf(stderr, "Failed to allocate zero-copy "
                    "buffer for transfer %d\nFalling "
                    "back to buffers in userspace\n", i);
            dev->use_zerocopy = 0;
            break;
        }
    }

    /* zero-copy buffer allocation failed (partially or completely)
     * we need to free the buffers again if already allocated */
    if (!dev->use_zerocopy) {
        for (i = 0; i < dev->xfer_buf_num; ++i) {
            if (dev->xfer_buf[i])
                libusb_dev_mem_free(dev->devh,
                            dev->xfer_buf[i],
                            dev->xfer_buf_len);
        }
    }
#endif

    /* no zero-copy available, allocate buffers in userspace */
    if (!dev->use_zerocopy) {
        for (i = 0; i < dev->xfer_buf_num; ++i) {
            dev->xfer_buf[i] = malloc(dev->xfer_buf_len);

            if (!dev->xfer_buf[i])
                return -ENOMEM;
        }
    }

    return 0;
}

static int _rtlsdr_free_async_buffers(rtlsdr_dev_t *dev)
{
    unsigned int i;

    if (!dev)
        return -1;

    if (dev->xfer) {
        for(i = 0; i < dev->xfer_buf_num; ++i) {
            if (dev->xfer[i]) {
                libusb_free_transfer(dev->xfer[i]);
            }
        }

        free(dev->xfer);
        dev->xfer = NULL;
    }

    if (dev->xfer_buf) {
        for (i = 0; i < dev->xfer_buf_num; ++i) {
            if (dev->xfer_buf[i]) {
                if (dev->use_zerocopy) {
#if defined (__linux__) && LIBUSB_API_VERSION >= 0x01000105
                    libusb_dev_mem_free(dev->devh,
                                dev->xfer_buf[i],
                                dev->xfer_buf_len);
#endif
                } else {
                    free(dev->xfer_buf[i]);
                }
            }
        }

        free(dev->xfer_buf);
        dev->xfer_buf = NULL;
    }

    return 0;
}

int rtlsdr_read_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, void *ctx,
              uint32_t buf_num, uint32_t buf_len)
{
    unsigned int i;
    int r = 0;
    struct timeval tv = { 1, 0 };
    struct timeval zerotv = { 0, 0 };
    enum qs1r_async_status next_status = QS1R_INACTIVE;

    if (!dev)
        return -1;

    if (QS1R_INACTIVE != dev->async_status)
        return -2;

    dev->async_status = QS1R_RUNNING;
    dev->async_cancel = 0;

    dev->cb = cb;
    dev->cb_ctx = ctx;

    if (buf_num > 0)
        dev->xfer_buf_num = buf_num;
    else
        dev->xfer_buf_num = DEFAULT_BUF_NUMBER;

    if (buf_len > 0 && buf_len % 512 == 0) /* len must be multiple of 512 */
        dev->xfer_buf_len = buf_len;
    else
        dev->xfer_buf_len = DEFAULT_BUF_LENGTH;

    _rtlsdr_alloc_async_buffers(dev);

    for(i = 0; i < dev->xfer_buf_num; ++i) {
        libusb_fill_bulk_transfer(dev->xfer[i],
                      dev->devh,
                      0x81,
                      dev->xfer_buf[i],
                      dev->xfer_buf_len,
                      _libusb_callback,
                      (void *)dev,
                      BULK_TIMEOUT);

        r = libusb_submit_transfer(dev->xfer[i]);
        if (r < 0) {
            fprintf(stderr, "Failed to submit transfer %i\n"
                    "Please increase your allowed " 
                    "usbfs buffer size with the "
                    "following command:\n"
                    "echo 0 > /sys/module/usbcore"
                    "/parameters/usbfs_memory_mb\n", i);
            dev->async_status = QS1R_CANCELING;
            break;
        }
    }

    while (QS1R_INACTIVE != dev->async_status) {
        r = libusb_handle_events_timeout_completed(dev->ctx, &tv,
                               &dev->async_cancel);
        if (r < 0) {
            /*fprintf(stderr, "handle_events returned: %d\n", r);*/
            if (r == LIBUSB_ERROR_INTERRUPTED) /* stray signal */
                continue;
            break;
        }

        if (QS1R_CANCELING == dev->async_status) {
            next_status = QS1R_INACTIVE;

            if (!dev->xfer)
                break;

            for(i = 0; i < dev->xfer_buf_num; ++i) {
                if (!dev->xfer[i])
                    continue;

                if (LIBUSB_TRANSFER_CANCELLED !=
                        dev->xfer[i]->status) {
                    r = libusb_cancel_transfer(dev->xfer[i]);
                    /* handle events after canceling
                     * to allow transfer status to
                     * propagate */
#ifdef _WIN32
                    Sleep(1);
#endif
                    libusb_handle_events_timeout_completed(dev->ctx,
                                           &zerotv, NULL);
                    if (r < 0)
                        continue;

                    next_status = QS1R_CANCELING;
                }
            }

            if (dev->dev_lost || QS1R_INACTIVE == next_status) {
                /* handle any events that still need to
                 * be handled before exiting after we
                 * just cancelled all transfers */
                libusb_handle_events_timeout_completed(dev->ctx,
                                       &zerotv, NULL);
                break;
            }
        }
    }

    _rtlsdr_free_async_buffers(dev);

    dev->async_status = next_status;

    return r;
}

int rtlsdr_cancel_async(rtlsdr_dev_t *dev)
{
    if (!dev)
        return -1;

    /* if streaming, try to cancel gracefully */
    if (QS1R_RUNNING == dev->async_status) {
        dev->async_status = QS1R_CANCELING;
        _async_cancel = 1;
        return 0;
    }

    /* if called while in pending state, change the state forcefully */
#if 0
    if (QS1R_INACTIVE != dev->async_status) {
        dev->async_status = QS1R_INACTIVE;
        return 0;
    }
#endif
    return -2;
}

