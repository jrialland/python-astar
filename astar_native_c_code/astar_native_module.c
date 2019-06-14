
#define PY_SSIZE_T_CLEAN

#include "astar_impl.h"
#include <Python.h>

typedef struct _py_astar_call_ctx {
	PyObject *neighbors_fn;
	PyObject *heuristic_cost_estimate_fn; 
	PyObject *distance_between_fn;
	PyObject *is_goal_reached_fn;
} py_astar_call_ctx_t;

static long py_hash_fn(void *obj) {
	return (long)PyObject_Hash((PyObject*)obj);
}

static void* py_neighbors_fn(void *invocation_ctx, void *node) {
	py_astar_call_ctx_t *ctx = (py_astar_call_ctx_t*)invocation_ctx;
	PyObject *l = PyObject_CallFunctionObjArgs(ctx->neighbors_fn, (PyObject*)node, NULL);
	PyObject * iter = PyObject_GetIter(l);
	Py_DECREF(l);
	return iter;
}

static void* py_iter_next_fn(void *invocation_ctx, void *iter) {
	PyObject* next = PyIter_Next((PyObject*)iter);
	return next;
}

static void py_iter_free_fn(void *invocation_ctx, void *iter) {
	Py_DECREF((PyObject*)iter);
}

static double py_heuristic_cost_estimate_fn(void *invocation_ctx, void *n, void *goal) {
	py_astar_call_ctx_t *ctx = (py_astar_call_ctx_t*)invocation_ctx;
	PyObject *result = PyObject_CallFunctionObjArgs(ctx->heuristic_cost_estimate_fn, (PyObject*)n, (PyObject*)goal, NULL);
	double d = PyFloat_AsDouble(result);
	Py_DECREF(result);
	return d;
}

static double py_distance_between_fn(void *invocation_ctx, void *n1, void *n2) {
	py_astar_call_ctx_t *ctx = (py_astar_call_ctx_t*)invocation_ctx;
	PyObject *result = PyObject_CallFunctionObjArgs(ctx->distance_between_fn, (PyObject*)n1, (PyObject*)n2, NULL);
	double d = PyFloat_AsDouble(result);
	Py_DECREF(result);
	return d;
}

static int py_is_goal_reached_fn(void *invocation_ctx, void *n, void *goal) {
	py_astar_call_ctx_t *ctx = (py_astar_call_ctx_t*)invocation_ctx;
	PyObject *result = PyObject_CallFunctionObjArgs(ctx->is_goal_reached_fn, (PyObject*)n, (PyObject*)goal, NULL);
	int isTrue = PyObject_IsTrue(result);
	Py_DECREF(result);
	return isTrue;
}

static PyObject* astar(PyObject *self, PyObject *args) {

	py_astar_call_ctx_t invocation_ctx;
	astar_param_t call_params;

	call_params.invocation_ctx = &invocation_ctx;
	call_params.hash_fn = py_hash_fn;
	call_params.heuristic_cost_estimate_fn = py_heuristic_cost_estimate_fn;
	call_params.distance_between_fn = py_distance_between_fn;
	call_params.is_goal_reached_fn = py_is_goal_reached_fn;
	call_params.neighbors_fn = py_neighbors_fn;
	call_params.iter_next_fn = py_iter_next_fn;
	call_params.iter_free_fn = py_iter_free_fn;

	astar_result_t astar_result;

	PyObject *py_param;
	if(!PyArg_ParseTuple(args, "O", &py_param)) {
		return NULL;
	}

	PyObject *start = PyObject_GetAttrString(py_param, "start");
	call_params.start = start;

	PyObject *goal = PyObject_GetAttrString(py_param, "goal");
	call_params.goal = goal;

	invocation_ctx.neighbors_fn = PyObject_GetAttrString(py_param, "neighbors_fn");

	invocation_ctx.heuristic_cost_estimate_fn = PyObject_GetAttrString(py_param, "heuristic_cost_estimate_fn");

	invocation_ctx.distance_between_fn = PyObject_GetAttrString(py_param, "distance_between_fn");

	invocation_ctx.is_goal_reached_fn = PyObject_GetAttrString(py_param, "is_goal_reached_fn");

	PyObject *oReversePath = PyObject_GetAttrString(py_param, "reverse_path");
	call_params.reverse_path = PyObject_IsTrue(oReversePath);
	Py_DECREF(oReversePath);
	Py_DECREF(py_param);

	astar_impl(&call_params, &astar_result);
	
	PyObject * lReturned = PyList_New(astar_result.size);
	size_t i;
	for(i=0; i < astar_result.size; i++) {
		PyList_SetItem(lReturned, i, astar_result.path[i]);	
	};

	if(astar_result.path != NULL) {
		free(astar_result.path);
	}

	return lReturned;
}

/*----------------------------------------------------------------------------*/
FILE *log_file = NULL;

static PyMethodDef astart_native_methods[] = {
	{"astar", astar, METH_VARARGS, "native astar implementation"},
	{NULL, NULL, 0, NULL}
};

static struct PyModuleDef astar_native_moduledef = {
	PyModuleDef_HEAD_INIT,
	"astar_native",
	"plain C implementation of the astar algorithm",
	-1,
	astart_native_methods
};

PyMODINIT_FUNC
PyInit_astar_native(void) {
	return PyModule_Create(&astar_native_moduledef);	
}
