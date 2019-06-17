
#include <stdlib.h>
#include <assert.h>

#include "priorityqueue.h"

int int_cmp(const void *pa, const void *pb) {
	int a = *((int*)pa);
	int b = *((int*)pb);
	int result = a == b ? 0 : (a < b ? -1 : 1);
	return result;
}

void test_pq_1() {
	
	pq_t *pq = pq_new(int_cmp);
	
	pq_push(pq, (void*)3);
	pq_push(pq, (void*)1);
	pq_push(pq, (void*)2);
	pq_push(pq, (void*)0);
	pq_push(pq, (void*)4);


	assert(0 == (int)pq_pop(pq));
	assert(1 == (int)pq_pop(pq));
	assert(2 == (int)pq_pop(pq));
	assert(3 == (int)pq_pop(pq));
	assert(4 == (int)pq_pop(pq));
	assert(false == pq_has_len(pq));
	pq_free(pq);
}

void test_pq_2() {
	int size = 512;
	pq_t *pq = pq_new(int_cmp);

	int *array1 = malloc(size* sizeof(int));
	int *array2 = malloc(size * sizeof(int));


	for(int i=0; i<size; i++) {
		array1[i] = rand() % 100;
		pq_push(pq, (void*)array1[i]);
	}	

	qsort(array1, size, sizeof(int), int_cmp);

	for(int i=0; i<size; i++) {
		array2[i] = (int)pq_pop(pq);
	}

	assert(pq_has_len(pq) == false);
	

	for(int i=0; i<size; i++) {
		assert(array1[i] == array2[i]);
	}
	free(array1);
	free(array2);
	pq_free(pq);
}

int main(void) {
	test_pq_1();
	for(int i=0; i < 10000; i++) {
		test_pq_2();
	}
	return 0;

}
