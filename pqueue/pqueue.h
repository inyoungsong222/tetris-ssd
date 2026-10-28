// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2014, Volkan Yazıcı <volkan.yazici@gmail.com>
 * All rights reserved.
 */

#ifndef PQUEUE_H
#define PQUEUE_H

typedef unsigned long long pqueue_pri_t;

typedef pqueue_pri_t (*pqueue_get_pri_f)(void *a);
typedef void (*pqueue_set_pri_f)(void *a, pqueue_pri_t pri);
typedef int (*pqueue_cmp_pri_f)(pqueue_pri_t next, pqueue_pri_t curr);

typedef size_t (*pqueue_get_pos_f)(void *a);
typedef void (*pqueue_set_pos_f)(void *a, size_t pos);

typedef struct pqueue_t {
	size_t size;
	size_t avail;
	size_t step;
	pqueue_cmp_pri_f cmppri;
	pqueue_get_pri_f getpri;
	pqueue_set_pri_f setpri;
	pqueue_get_pos_f getpos;
	pqueue_set_pos_f setpos;
	void **d;
} pqueue_t;

pqueue_t *pqueue_init(size_t n, pqueue_cmp_pri_f cmppri, pqueue_get_pri_f getpri,
		      pqueue_set_pri_f setpri, pqueue_get_pos_f getpos, pqueue_set_pos_f setpos);

void pqueue_free(pqueue_t *q);

size_t pqueue_size(pqueue_t *q);

void *pqueue_peek_at(pqueue_t *q, size_t i);

int pqueue_insert(pqueue_t *q, void *d);

void pqueue_change_priority(pqueue_t *q, pqueue_pri_t new_pri, void *d);

void *pqueue_pop(pqueue_t *q);

int pqueue_remove(pqueue_t *q, void *d);

void *pqueue_peek(pqueue_t *q);

int pqueue_is_valid(pqueue_t *q);

#endif

