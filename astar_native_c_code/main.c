
#include "astar_impl.h"
#include <stdio.h>
FILE * log_file;

int main(void) {


	searchnode_t * a = searchnode_new(1);
	a->fscore = 1.0;

	searchnode_t * b = searchnode_new(2);
	b->fscore = 2.0;

	searchnode_t * c = searchnode_new(3);
	c->fscore = 3.0;

	searchnode_t * d = searchnode_new(4);
	d->fscore = 4.0;


	searchnode_pq_t * pq = searchnode_pq_new();
	searchnode_pq_push(pq, a);
	searchnode_pq_push(pq, b);
	searchnode_pq_push(pq, c);
	searchnode_pq_push(pq, d);

/*
	searchnode_pq_push(pq, d);
	searchnode_pq_push(pq, c);
	searchnode_pq_push(pq, b);
	searchnode_pq_push(pq, a);
*/

	while(searchnode_pq_has_len(pq)) {
		printf("%f\n", searchnode_pq_pop(pq)->fscore);
	}

	searchnode_free(pq);

	return 0;


}
