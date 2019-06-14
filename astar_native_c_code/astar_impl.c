
#include <math.h>
#include "astar_impl.h"
#include "rbtree.h"

searchnode_t *searchnode_new(void *data) {
    searchnode_t *n = malloc(sizeof(searchnode_t));
    n->data = data;
    n->came_from = NULL;
    n->gscore = INFINITY;
    n->fscore = INFINITY;
    n->closed = false;
    n->out_openset = true;
    return n;
}

void searchnode_free(searchnode_t *n) {
	free(n);
}

searchnode_pq_t *searchnode_pq_new() {
	searchnode_pq_t *pq = malloc(sizeof(searchnode_pq_t));
	pq->items = NULL;
	pq->size = 0;
	return pq;
}

void searchnode_pq_free(searchnode_pq_t *pq) {
	if(pq->items != NULL) {
		free(pq->items);
	}
	free(pq);
}

int searchnode_pq_has_len(searchnode_pq_t *pq) {
	return pq->size > 0;
}

int searchnode_pq_cmp(const void *a, const void* b) {
    searchnode_t **pa = (searchnode_t**)a, **pb = (searchnode_t**)b;
    float fa = (*pa)->fscore;
    float fb = (*pb)->fscore;
    if(fa == fb) {
        return 0;
    }
    return fa > fb ? -1 : 1;
}

void searchnode_pq_push(searchnode_pq_t *pq, searchnode_t *item) {
	pq->size += 1;
	if(pq->items == NULL) {
		pq->items = malloc(pq->size * sizeof(searchnode_t*));
	} else {
		pq->items = realloc(pq->items, pq->size * sizeof(searchnode_t *));
	}
	pq->items[pq->size -1] = item;
}

searchnode_t *searchnode_pq_pop(searchnode_pq_t *pq) {
        qsort(pq->items, pq->size, sizeof(searchnode_t *), searchnode_pq_cmp);
	pq->size -= 1;
	searchnode_t *popped = pq->items[pq->size];
        if(pq->size == 0) {
                free(pq->items);
		pq->items = NULL;
	} else {
		pq->items = realloc(pq->items, pq->size * sizeof(searchnode_t*));
	}
	return popped;
}


static void 
astar_impl_feed_result(
	searchnode_t *current,
 	searchnode_t *goal,
	int reverse_path,
	astar_result_t *result
) {
	int capacity = 2;
	result->size = 0;
	result->path = malloc(capacity * sizeof(void*));
	result->path[result->size++] = current->data;

	while(current->came_from != NULL) {
		current = current->came_from;
		if(result->size == capacity) {
			capacity += ASTAR_ALLOC_INCREMENT;
			result->path = realloc(result->path, capacity * sizeof(void*)); 
		}
		result->path[result->size++] = current->data;
	}

	result->path = realloc(result->path, result->size * sizeof(void*));
	
	if(!reverse_path) {
		int i = result->size -1;
		int j = 0;
		while(i > j) {
			void *tmp = result->path[i];
			result->path[i--] = result->path[j];
			result->path[j++] = tmp;
		}
	}
}

#if __SIZEOF_POINTER__ <= __SIZEOF_LONG__
static long default_hash_fn(void *p) {
	return (long)p;
}

int voidptr_key_compare(void *n1, void *n2, void *opt) {
	return (long)n1 - (long)n2;
}

#else
    #error "Will no be able to convert pointers to longs because sizeof(void*) > sizeof(long) !"
#endif


void searchnode_free_entry(void *key, void*value, void* opt) {
	searchnode_free((searchnode_t*)value);
}

void astar_impl(astar_param_t* param, astar_result_t *result) {

	// provide a dummy hash function if missing
	if(param->hash_fn == NULL) {
		param->hash_fn= default_hash_fn;
	}

	//default result if not found
	result->path = NULL;
	result->size = 0;

	//if goal is reached on step 1, return immediately
	if(param->is_goal_reached_fn(param->invocation_ctx, param->start, param->goal)) {
		result->path = malloc(sizeof(void*));
		result->path[0] = param->start;
		result->size = 1;
		return;
	}
	
	searchnode_t *start_node = searchnode_new(param->start);	
	searchnode_t *goal_node = searchnode_new(param->goal);

	// create a map of all search node
	rbtree_tree *nodes = rbtree_create(voidptr_key_compare, NULL);

	rbtree_insert(nodes, (void*)param->hash_fn(start_node->data), start_node);
	rbtree_insert(nodes, (void*)param->hash_fn(goal_node->data), goal_node);

	//create the priority queue
	searchnode_pq_t *openset = searchnode_pq_new();

	start_node->gscore = .0;
	start_node->fscore = param->heuristic_cost_estimate_fn(param->invocation_ctx, start_node->data, goal_node->data);
	searchnode_pq_push(openset, start_node);
	start_node->out_openset = false;

	//while openset is not empty
	while(searchnode_pq_has_len(openset)) {

		searchnode_t * current = searchnode_pq_pop(openset);

		//test if the goal is reached
		if(param->is_goal_reached_fn(param->invocation_ctx, current->data, goal_node->data)) {
			astar_impl_feed_result(current, goal_node, param->reverse_path, result);
			break;
		}

		current->out_openset = true;
		current->closed = true;

		//for each neighbor of the current node
		void *neighbor_data;
		void *iterator = param->neighbors_fn(param->invocation_ctx, current->data);

		while( (neighbor_data = param->iter_next_fn(param->invocation_ctx, iterator)) != NULL) {
			
			//get the search node associated with the neighbor
			long hash = param->hash_fn(neighbor_data);
			searchnode_t * neighbor = rbtree_lookup(nodes, (void*)hash);
			if(neighbor == NULL) {
				neighbor = searchnode_new(neighbor_data);
				rbtree_insert(nodes, (void*)hash, neighbor);
			}

			if(neighbor->closed) {
				continue;
			}

			double tentative_g_score = current->gscore + param->distance_between_fn(param->invocation_ctx, current->data, neighbor->data);

			if(tentative_g_score >= neighbor->gscore) {
				continue;
			}
			
			neighbor->came_from = current;
			neighbor->gscore = tentative_g_score;
			neighbor->fscore = tentative_g_score + param->heuristic_cost_estimate_fn(param->invocation_ctx, neighbor->data, goal_node->data);

			if(neighbor->out_openset) {
				neighbor->out_openset = false;
				searchnode_pq_push(openset, neighbor);
			}
		}

		//dealloc the iterator
		param->iter_free_fn(param->invocation_ctx, iterator);
	}

	//dealloc all nodes
	rbtree_foreach(nodes, searchnode_free_entry, NULL);
	rbtree_delete(nodes);

	//dealloc the priority queue
	searchnode_pq_free(openset);
}

