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

All queries share one environment. Each pair of endpoints comes from
`Environment.random_query()` and is used by both planners. PRM builds its roadmap
once and removes the temporary start/goal nodes after each query; RRT builds fresh
trees for each query. Each figure compares the two paths after shortcutting.

The defaults are three queries and seed 4. Use `--queries 1` for a single query.

The console prints PRM's one-time setup cost and each planner's query time in
milliseconds, alongside whether a path was found. Query times exclude shortcutting
and plotting. For a single-query comparison, add PRM's setup cost to its query time.
