import argparse
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


def plan_prm_query(graph, start, goal):
    roadmap_size = graph.graph_size
    try:
        graph.add_node(*start)
        graph.add_node(*goal)
        return graph.bfs(start, goal)
    finally:
        while graph.graph_size > roadmap_size:
            graph.remove_last_node()


def plan_rrt_query(env, start, goal, radius = 2.0, max_rep = 500):
    rrt_start = RRT_Tree(env, *start, radius)
    rrt_goal = RRT_Tree(env, *goal, radius)
    mid_start = None
    mid_goal = None

    for _ in range(max_rep):
        x_rand = np.random.rand() * env.size_x
        y_rand = np.random.rand() * env.size_y

        nearest_node = rrt_start.find_nearest(x_rand, y_rand)
        if rrt_start.match_node(nearest_node, x_rand, y_rand):
            mid_goal = rrt_goal.scan_node(rrt_start.nodes[-1], radius)
            if mid_goal is not None:
                mid_start = rrt_start.nodes[-1]
                break

        nearest_node = rrt_goal.find_nearest(x_rand, y_rand)
        if rrt_goal.match_node(nearest_node, x_rand, y_rand):
            mid_start = rrt_start.scan_node(rrt_goal.nodes[-1], radius)
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

    rrt_samples = [(node.x, node.y) for node in rrt_start.nodes + rrt_goal.nodes]
    return list(rrt_path_trace), rrt_samples


def main(num_queries = 3, seed = 4):
    if num_queries < 1:
        raise ValueError("num_queries must be at least 1.")

    np.random.seed(seed)
    env = Environment(10, 6, 5)
    graph = Graph(env, 500, 2.0)
    graph.build_graph()
    print(f"Built one PRM roadmap with {graph.graph_size} nodes for {num_queries} queries.")

    plotted_queries = 0
    for query_index in range(num_queries):
        query = env.random_query()
        if query is None:
            print(f"Query {query_index + 1}: could not generate endpoints; skipped.")
            continue
        x_start, y_start, x_goal, y_goal = query
        start, goal = (x_start, y_start), (x_goal, y_goal)

        prm_path_trace = plan_prm_query(graph, start, goal)
        prm_path_trace = path_shortcutting(prm_path_trace, 1000, env)

        rrt_path_trace, rrt_samples = plan_rrt_query(env, start, goal)
        rrt_path_trace = path_shortcutting(rrt_path_trace, 1000, env)

        prm_status = "path found" if prm_path_trace else "no path found"
        rrt_status = "path found" if rrt_path_trace else "no path found"
        print(f"Query {query_index + 1}: PRM {prm_status}; RRT {rrt_status}.")

        figure, axes = pl.subplots(1, 2, figsize=(12, 5))
        figure.suptitle(f"Query {query_index + 1}/{num_queries}")
        plot_path(env, prm_path_trace, start, goal, title="PRM",
                  samples=graph.G, ax=axes[0])
        plot_path(env, rrt_path_trace, start, goal, title="RRT",
                  samples=rrt_samples, ax=axes[1])
        figure.tight_layout()
        plotted_queries += 1

    if plotted_queries:
        pl.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Compare PRM and RRT over one environment.")
    parser.add_argument("--queries", type = int, default = 3, help="Number of random queries (default: 3).")
    parser.add_argument("--seed", type = int, default = 4, help="Random seed (default: 4).")
    args = parser.parse_args()
    if args.queries < 1:
        parser.error("--queries must be at least 1")
    main(num_queries = args.queries, seed = args.seed)
