from collections import deque

import numpy as np
import pylab as pl

from planning.environment import Environment
from planning.prm import Graph
from planning.rrt import RRT_Tree
from planning.path_shortcutting import path_shortcutting


def plot_path(env, path_trace, start, goal, *, title = "Path", samples = None, ax = None):
    if ax is None:
        _, ax = pl.subplots()
    pl.sca(ax)
    env.plot()

    if samples is not None and len(samples) > 0:
        sample_x, sample_y = zip(*samples)
        ax.plot(sample_x, sample_y, ".", color="0.7", markersize=3)

    if path_trace is not None and len(path_trace) > 0:
        path_x, path_y = zip(*path_trace)
        ax.plot(path_x, path_y, "g.-", linewidth=2, markersize=4)
    else:
        title += " (no path found)"

    env.plot_query(*start, *goal)
    ax.set_title(title)
    ax.set_xlim(0, env.size_x)
    ax.set_ylim(0, env.size_y)
    ax.set_aspect("equal", adjustable="box")
    return ax


def main():
    np.random.seed(4)
    env = Environment(10, 6, 5)
    query = env.random_query()
    if query is None:
        print("Could not generate a collision-free start and goal.")
        return
    x_start, y_start, x_goal, y_goal = query
    start, goal = (x_start, y_start), (x_goal, y_goal)

    graph = Graph(env, 500, 2.0)
    graph.build_graph()
    graph.add_node(*start)
    graph.add_node(*goal)
    prm_path_trace = graph.bfs(start, goal)
    prm_path_trace = path_shortcutting(prm_path_trace, 1000, env)
    graph.remove_last_node()
    graph.remove_last_node()

    rrt_start = RRT_Tree(env, *start, 2.0)
    rrt_goal = RRT_Tree(env, *goal, 2.0)
    max_rep = 500
    mid_start = None
    mid_goal = None

    for _ in range(max_rep):
        x_rand = np.random.rand() * env.size_x
        y_rand = np.random.rand() * env.size_y

        nearest_node = rrt_start.find_nearest(x_rand, y_rand)
        if rrt_start.match_node(nearest_node, x_rand, y_rand):
            mid_goal = rrt_goal.scan_node(rrt_start.nodes[-1], 2)
            if mid_goal is not None:
                mid_start = rrt_start.nodes[-1]
                break

        nearest_node = rrt_goal.find_nearest(x_rand, y_rand)
        if rrt_goal.match_node(nearest_node, x_rand, y_rand):
            mid_start = rrt_start.scan_node(rrt_goal.nodes[-1], 2)
            if mid_start is not None:
                mid_goal = rrt_goal.nodes[-1]
                break

    rrt_path_trace = deque()
    if mid_start is not None and mid_goal is not None:
        while mid_start != rrt_start.Root:
            rrt_path_trace.appendleft((mid_start.x, mid_start.y))
            mid_start = mid_start.parent
        while mid_goal != rrt_goal.Root:
            rrt_path_trace.append((mid_goal.x, mid_goal.y))
            mid_goal = mid_goal.parent
        rrt_path_trace.appendleft(start)
        rrt_path_trace.append(goal)

    rrt_path_trace = path_shortcutting(list(rrt_path_trace), 1000, env)

    figure, axes = pl.subplots(1, 2, figsize=(12, 5))
    plot_path(env, prm_path_trace, start, goal, title = "PRM",
              samples=graph.G, ax=axes[0])
    rrt_samples = [(node.x, node.y) for node in rrt_start.nodes + rrt_goal.nodes]
    plot_path(env, rrt_path_trace, start, goal, title = "RRT",
              samples=rrt_samples, ax=axes[1])
    figure.tight_layout()
    pl.show()


if __name__ == "__main__":
    main()
