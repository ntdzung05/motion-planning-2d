import numpy as np


def path_shortcutting(trace, maxrep, env):
  trace = list(trace)
  for _ in range(maxrep):
    if len(trace) < 3:
      break

    u = np.random.randint(0, len(trace) - 2)
    v = np.random.randint(u + 1, len(trace) - 1)

    mid_u = np.random.uniform(0, 1)
    mid_v = np.random.uniform(0, 1)

    p_x = trace[u][0] + mid_u * (trace[u + 1][0] - trace[u][0])
    p_y = trace[u][1] + mid_u * (trace[u + 1][1] - trace[u][1])

    q_x = trace[v][0] + mid_v * (trace[v + 1][0] - trace[v][0])
    q_y = trace[v][1] + mid_v * (trace[v + 1][1] - trace[v][1])

    if (not env.check_collision(p_x, p_y)
        and not env.check_collision(q_x, q_y)
        and not env.pass_obstacle(p_x, p_y, q_x, q_y)):
      trace = trace[:u + 1] + [(p_x, p_y), (q_x, q_y)] + trace[v + 1:]
  return trace
