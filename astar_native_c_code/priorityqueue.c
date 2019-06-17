
#include <stdlib.h>
#include "priorityqueue.h"

pq_t *pq_new(pq_cmp_fn_t compare_fn) {
	pq_t *pq = malloc(sizeof(pq_t));
	mp_create_fixed_pool(&pq->mp,"Priority Queue");
        pq->compare_fn = compare_fn;
        pq->first = NULL;
	return pq;
}

void pq_free(pq_t *pq) {
     mp_free_pool(&pq->mp);
     free(pq);
}

void pq_push(pq_t *pq, void *data) {
	pq_cell_t *inserted = mp_alloc_n0(&pq->mp,sizeof(pq_cell_t));
	inserted->data = data;
	inserted->next = NULL;

	if(pq->first == NULL) {
		pq->first = inserted;
		return;
	}

	pq_cell_t *current = pq->first, *prec = NULL;
	while(current != NULL && pq->compare_fn(&data, &(current->data)) > 0) {
		prec = current;
		current = current->next;
	}
	inserted->next = current;
	if(prec != NULL) {
		prec->next = inserted;
	} else {
		pq->first = inserted;
	}
}

void *pq_pop(pq_t *pq) { 
	if(pq->first == NULL) {
		return NULL;
        }
	pq_cell_t *current = pq->first;
	void *data = current->data;
        pq->first = current->next;
	mp_free(current);
	return data;	
}

bool pq_has_len(pq_t *pq) {
	return  pq->first != NULL;
}

