#ifndef ASTAR_IMPL_H_
#define ASTAR_IMPL_H_

#include <stdlib.h>

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
