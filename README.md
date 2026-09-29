# motion-planning-2d

## Setup

Install Python 3 (tested with Python 3.12) and open a terminal in the project root.

Windows / PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe demo.py --queries 3 --seed 4
```

macOS / Linux:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python demo.py --queries 3 --seed 4
```

These commands use the virtual environment directly; activation is optional.
Dependencies are NumPy and Matplotlib.

## Demo

All queries share one environment. By default, each pair of endpoints comes from
`Environment.random_query()` and is used by both planners. PRM builds its roadmap
once and removes the temporary start/goal nodes after each query; RRT builds fresh
trees for each query. Each figure compares the two paths after shortcutting.

The defaults are three queries and seed 4. Use `--queries 1` for a single query.

To provide custom endpoints, pass four coordinates. They are checked by
`Environment.query()`. Each supplied pair runs once.

```powershell
python demo.py --queries 1 --seed 4 --query 0.5 0.5 9.5 5.5
```

Or call the demo from Python:

```python
from demo import main
main(num_queries=1, seed=4, custom_queries=[(0.5, 0.5, 9.5, 5.5)])
```

For multiple custom queries, repeat `--query`. This runs two custom queries
followed by two random queries:

```powershell
python demo.py --queries 4 --seed 4 --query 0.5 0.5 9.5 5.5 --query 0.5 5.5 9.5 0.5
```

Or pass a list to `main()`:

```python
main(num_queries=4, seed=4, custom_queries=[
    (0.5, 0.5, 9.5, 5.5),
    (0.5, 5.5, 9.5, 0.5),
])
```

`num_queries` / `--queries` sets the total number of query slots. Custom pairs run
first, in list order; remaining slots call `Environment.random_query()`. An empty
custom list makes every query random. The custom list cannot exceed the total.
All entries use the same environment and PRM roadmap. Rejected
endpoint pairs are skipped, and the remaining queries still run. In Python,
an entry of `None` generates a random query, allowing custom and random queries
in the same list. To repeat a custom pair, include it multiple times in the list.

The console prints PRM's one-time setup cost and each planner's query time in
milliseconds, alongside whether a path was found. Query times exclude shortcutting
and plotting. For a single-query comparison, add PRM's setup cost to its query time.

## Single-query and multi-query performance

Measured locally on an Intel Core i7-12650H running Windows 11, with Python 3.12.6,
NumPy 2.5.3, and Matplotlib 3.11.2. Each batch uses one environment and one PRM
roadmap; RRT starts fresh for each query. Settings: 10-by-6 workspace, five
triangles, up to 500 PRM nodes, radius 2, and a 500-iteration RRT budget.

The table averages three timing runs for each of seeds 0–4, after one warm-up.
Planning totals include unsuccessful searches and PRM's one-time roadmap setup.
Endpoint generation, shortcutting, and plotting are excluded. The measurements
used the demo's planning helpers without figures, retaining the normal random
query and shortcutting calls between planning steps.

| Queries per environment | PRM total (ms) | RRT total (ms) | PRM per query (ms) | RRT per query (ms) | PRM paths found | RRT paths found |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 147.58 | 7.79 | 147.58 | 7.79 | 5/5 | 5/5 |
| 5 | 163.90 | 28.14 | 32.78 | 5.63 | 25/25 | 24/25 |
| 20 | 214.63 | 139.99 | 10.73 | 7.00 | 100/100 | 91/100 |

Totals are averages per batch. Per-query costs divide those totals by the batch
size, distributing PRM setup across the queries. Success counts cover distinct
seed/query pairs across the five environments; repeated timing runs are not
counted as additional queries. All three repetitions had the same outcomes.

- **Single query:** RRT was faster because it avoided the roadmap setup cost.
- **Multiple queries:** PRM's average cost fell from 147.58 to 10.73 ms per query
  as the roadmap was reused. Its search cost alone averaged 3.58–3.95 ms per
  query, with about 143–145 ms spent building each roadmap.
- **Tradeoff in this sample:** RRT remained faster overall at 20 queries, but
  returned paths for 91/100 queries versus PRM's 100/100. A failed RRT search
  means no path was found within its iteration budget, not proof that none exists.

These are small local measurements, not a universal ranking. Different seeds,
obstacles, planner settings, and machine load can change the results. Use
`--queries 1`, `--queries 5`, or `--queries 20` with a fixed `--seed` to compare
individual batches using the demo's timing output.
