/* Opt-in snapshot of one failed model in the isolated planner only.
 * Optimization and its return value are unchanged. No PX4 interaction. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "gurobi_c.h"
static int (*real_optimize)(GRBmodel *);
static int (*get_status)(GRBmodel *, const char *, int *);
static int (*write_model)(GRBmodel *, const char *);
static pthread_once_t once = PTHREAD_ONCE_INIT;
static atomic_flag captured = ATOMIC_FLAG_INIT;
static void resolve(void) {
  real_optimize = dlsym(RTLD_NEXT, "GRBoptimize");
  get_status = dlsym(RTLD_NEXT, "GRBgetintattr");
  write_model = dlsym(RTLD_NEXT, "GRBwrite");
  if (!real_optimize || !get_status || !write_model) {
    fputs("SANDO diagnostics: cannot resolve Gurobi symbols\n", stderr);
    _exit(126);
  }
}
int GRBoptimize(GRBmodel *model) {
  pthread_once(&once, resolve);
  int result = real_optimize(model);
  const char *path = getenv("SANDO_SOLVER_SNAPSHOT");
  int status = 0;
  if (path && !result && !get_status(model, GRB_INT_ATTR_STATUS, &status) &&
      (status == GRB_INFEASIBLE || status == GRB_INF_OR_UNBD) &&
      !atomic_flag_test_and_set(&captured)) {
    char temporary[4096];
    snprintf(temporary, sizeof(temporary), "%s.pending.lp", path);
    int error = write_model(model, temporary);
    if (!error) error = rename(temporary, path);
    fprintf(stderr, "SANDO diagnostics: model status=%d snapshot=%s error=%d\n", status, path, error);
  }
  return result;
}
