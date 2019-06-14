
#ifndef ASTAR_IMPL_H_
#define ASTAR_IMPL_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define ASTAR_ALLOC_INCREMENT 1024

typedef struct _searchnode {
	void *data;
	struct _searchnode *came_from;
	double gscore;
	double fscore;
	bool closed;
	bool out_openset;
} searchnode_t;

searchnode_t *searchnode_new(void *data);
void searchnode_free(searchnode_t *n);

typedef struct _searchnode_pq {
	searchnode_t **items;
	size_t size;
}searchnode_pq_t;

searchnode_pq_t *searchnode_pq_new(void);

void searchnode_pq_free(searchnode_pq_t *pq);
int searchnode_pq_has_len(searchnode_pq_t *pq); 
void searchnode_pq_push(searchnode_pq_t *pq, searchnode_t *item);
searchnode_t *searchnode_pq_pop(searchnode_pq_t *pq);

typedef long (*hash_fn_t)(void*);
typedef void* (*neighbors_fn_t)(void *invocation_ctx, void *node);
typedef void* (*iter_next_fn_t)(void *invocation_ctx, void *iterator);
typedef void (*iter_free_fn_t)(void *invocation_ctx, void *iterator);

typedef double (*heuristic_cost_estimate_fn_t)(void *invocation_ctx, void *n, void *goal);
typedef double (*distance_between_fn_t)(void *invocation_ctx, void *n1, void *n2);
typedef int (*is_goal_reached_fn_t)(void *invocation_ctx, void *n, void *goal);

typedef struct _astar_param {
    void *start;
    void *goal;
    int reverse_path;
    void * invocation_ctx;
    hash_fn_t hash_fn;
    heuristic_cost_estimate_fn_t heuristic_cost_estimate_fn;
    distance_between_fn_t distance_between_fn;
    is_goal_reached_fn_t is_goal_reached_fn;
    neighbors_fn_t neighbors_fn;
    iter_next_fn_t iter_next_fn;
    iter_free_fn_t iter_free_fn;
} astar_param_t;

typedef struct _astar_result {
	void ** path;
	size_t size;
} astar_result_t;

void astar_impl(astar_param_t* params, astar_result_t *result);

#endif/*ASTAR_IMPL_H_*/
