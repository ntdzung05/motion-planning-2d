# motion-planning-2d

Run the demo from the project root with NumPy and Matplotlib installed:

```powershell
python demo.py --queries 3 --seed 4
```

All queries share one environment. Each pair of endpoints comes from
`Environment.random_query()` and is used by both planners. PRM builds its roadmap
once and removes the temporary start/goal nodes after each query; RRT builds fresh
trees for each query. Each figure compares the two paths after shortcutting.

The defaults are three queries and seed 4. Use `--queries 1` for a single query.
