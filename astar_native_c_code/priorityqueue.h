#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_

#include <stdint.h>
#include <stdbool.h>
#include "mempool.h"

typedef int (*pq_cmp_fn_t)(const void*, const void*);

typedef struct _pq_cell {
	void * data;
        struct _pq_cell *next;
} pq_cell_t;

typedef struct _pq {
    pq_cmp_fn_t compare_fn;
    mempool_t mp;
    pq_cell_t *first;
} pq_t;

pq_t *pq_new(pq_cmp_fn_t compare_fn);

void pq_free(pq_t *pq);

void pq_push(pq_t *pq, void *data);

void *pq_pop(pq_t *pq);

bool pq_has_len(pq_t *pq);


#endif/*PRIORITYQUEUE_H_*/
