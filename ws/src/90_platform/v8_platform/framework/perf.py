"""F7 startup-only overhead benchmark; never used in the 500 Hz send path."""
import time

import graph as graph_mod
import render


def benchmark(graph, catalog, iterations=50):
    samples = []
    for _ in range(iterations):
        started = time.perf_counter()
        report = graph_mod.check(graph, catalog)
        render.render_plan(graph, catalog, report)
        samples.append((time.perf_counter() - started) * 1e3)
    ordered = sorted(samples)
    return {'iterations': iterations,
            'p50_ms': round(ordered[len(ordered) // 2], 3),
            'p95_ms': round(ordered[min(len(ordered) - 1, int(len(ordered) * .95))], 3),
            'max_ms': round(ordered[-1], 3),
            'startup_only': True,
            'note': 'check+plan runs before launch; the 500 Hz command path is the boundary adapter'}
