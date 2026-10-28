/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * nRF52833 USB driver ABI (CyberTech Labs)
 * Copyright (C) 2026 CyberTech Labs Ltd.
 *
 * VID 0x2FE9 / PID 0x0100 USB peripheral module.
 *
 * The module exposes a char device /dev/nrfN with:
 *   - poll() + read()  — telemetry frames (ISO IN or bulk IN stream);
 *   - write()          — one command via the interrupt OUT endpoint;
 *   - ioctl()          — one-shot bulk and vendor control transfers.
 *
 * This header is the single source of truth for the user/kernel ABI.
 * It may be included both by the kernel driver and by userspace
 * applications (it is installed to /usr/include/linux/ by
 * "make headers_install").
 */

#ifndef _LINUX_NRF_USB_H
#define _LINUX_NRF_USB_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define NRF_VID			0x2fe9
#define NRF_PID			0x0100

#define NRF_IOCTL_BASE		0xFE

/* Maximum size, in bytes, of a single telemetry frame returned by read(). */
#define NRF_TELEMETRY_SIZE	128

/*
 * Synchronous vendor control transfer (host → device, with optional
 * response). The 'data' buffer holds 'wLength' bytes on the way out and
 * is filled with the device response ('wLength' bytes) on the way back.
 */
struct nrf_ctrl {
	__u8	bRequest;
	__u8	bmRequestType;
	__u16	wValue;
	__u16	wIndex;
	__u16	wLength;
	__s32	timeout;
	__u8	data[256];
};

/* One-shot bulk OUT transfer (host → device). */
struct nrf_bulk {
	void	*data;
	__u16	length;
};

#define NRF_IO_CTRL		_IOWR(NRF_IOCTL_BASE, 0, struct nrf_ctrl)
#define NRF_IO_BULK		_IOW(NRF_IOCTL_BASE, 1, struct nrf_bulk)

#endif /* _LINUX_NRF_USB_H */
