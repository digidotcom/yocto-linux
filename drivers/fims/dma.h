/* -*- linux-c -*-
 *
 * drivers/fims/dma.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 *  !Revision:   $Revision: 1.3 $
 *  !Author:     Luis Galdos
 *  !Descr:
 *  !References:
 */


#ifndef __FIM_API_DMA_H
#define __FIM_API_DMA_H


#include <linux/module.h>
#include <linux/errno.h>

#include <mach/dma-ns921x.h>


/*
 * The access to the DMA-channels of the FIMs is only provided by the FIM-API
 * The FIM-drivers should not use the functions of this source code file
 */


inline static void fim_dma_reset_fifo(struct iohub_dma_fifo_t *fifo)
{
	if (!fifo)
		return;

	fifo->next_free = fifo->first;
	fifo->dma_first = fifo->first;
	fifo->dma_last = fifo->dma_next = NULL;
}


inline int fim_dma_init_fifo(struct iohub_dma_fifo_t *fifo, int length,
			     struct iohub_dma_desc_t **descs)
{
	if (!fifo || !length || !descs)
		return -EINVAL;

	/* @XXX: These are the start values for the API */
	fifo->length = length;
	fifo->first = (struct iohub_dma_desc_t *)descs;
	fifo->last = fifo->first + fifo->length - 1;

	fifo->next_free = fifo->first;
	fifo->dma_first = fifo->first;
	fifo->dma_last = fifo->dma_next = NULL;
	return 0;
}


inline static struct iohub_dma_desc_t *fim_dma_get_next(struct iohub_dma_fifo_t *fifo,
							struct iohub_dma_desc_t *ptr)
{
	if (ptr == fifo->last)
		return fifo->first;
	else
		return ptr + 1;
}


/*
 * Return the index of a passed DMA-descriptor
 */
inline static struct iohub_dma_desc_t *fim_dma_get_by_index(struct iohub_dma_fifo_t *fifo,
							    int index)
{
	if (index < 0)
		return fifo->first;
	else if (index >= fifo->length)
		return fifo->last;
	else
		return fifo->first + index;
}


inline static int fim_dma_length(struct iohub_dma_fifo_t *fifo)
{
	return fifo->length;
}


/*
 * Returns the number of free DMA-descriptors
 */
inline static int fim_dma_frees(struct iohub_dma_fifo_t *fifo)
{
	int retval;

	/* The first is the INIT state of the FIFO */
	if (!fifo->dma_last && !fifo->dma_next)
		retval = fifo->length;
	else if (fifo->dma_last == fifo->dma_next)
		retval = fifo->length;
	else if (!fifo->dma_last)
		retval = fifo->last - fifo->dma_next;
	else if (fifo->dma_next == fifo->dma_first)
		retval = (fifo->last - fifo->first);
	else if (fifo->dma_next > fifo->dma_first)
		retval = (fifo->last - fifo->dma_next) +
			(fifo->dma_first - fifo->first);
	else
		retval = (fifo->dma_first - fifo->dma_next);

	return retval;
}


/*
 * Check if the FIFO contains descriptors or is empty
 */
inline static int fim_dma_is_empty(struct iohub_dma_fifo_t *fifo)
{
	/* @XXX: That's only for FIFOs with one pointer */
	if (fifo->length == 1 && fifo->dma_next)
		return 0;

	if (fim_dma_frees(fifo) == fifo->length)
		return 1;
	else
		return 0;
}


inline static int fim_dma_get_index(struct iohub_dma_fifo_t *fifo,
				    struct iohub_dma_desc_t *desc)
{
	return desc - fifo->first;
}


/*
 * Check if there is enough space for a specific number of buffer descriptors.
 * Return a pointer to the first buffer descriptor is the FIFO has enough space
 */
inline static struct iohub_dma_desc_t *fim_dma_alloc(struct iohub_dma_fifo_t *fifo,
						     int count)
{
	int len, diff;
	struct iohub_dma_desc_t *tmp, *retval;

	len = fim_dma_frees(fifo);
	if (len < count)
		return NULL;

	if (!fifo->dma_next) {
		retval = fifo->first;
		fifo->dma_next = fifo->first + count - 1;
	} else {
		retval = fim_dma_get_next(fifo, fifo->dma_next);
		tmp = fifo->dma_next + count;
		if (tmp <= fifo->last)
			fifo->dma_next = tmp;
		else {
			diff = tmp - fifo->last - 1;
			fifo->dma_next = fifo->first + diff;
		}
	}

	return retval;
}



#endif /* ifndef __FIM_API_DMA_H */
