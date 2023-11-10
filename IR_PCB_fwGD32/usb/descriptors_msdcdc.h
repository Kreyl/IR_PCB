/*
 * descriptors_msdcdc.h
 *
 *  Created on: 2023.11.10
 *      Author: Kreyl
 */

#ifndef DESCRIPTORS_MSD_CDC_H___
#define DESCRIPTORS_MSD_CDC_H___

// Endpoints to be used for CDC
#define EP_CDC_DATA_IN      1
#define EP_CDC_DATA_OUT     1
#define EP_CDC_INTERRUPT    2

// Endpoints to be used for MSD
#define EP_MSD_DATA_IN      3
#define EP_MSD_DATA_OUT     3

// Endpoint Sizes for Full-Speed devices
#define EP_INTERRUPT_SZ     8   // Max size is 64 bytes
#define EP_CDC_BULK_SZ      64
#define EP_MSD_BULK_SZ      64

#endif // DESCRIPTORS_MSD_CDC_H___
