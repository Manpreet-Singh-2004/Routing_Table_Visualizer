from flask import Flask, render_template, request, jsonify
import heapq
from collections import defaultdict
import time

app = Flask(__name__)


NETWORK_TOPOLOGY = {
    ("A", "B"): 1,
    ("B", "A"): 1,
    ("A", "C"): 4,
    ("C", "A"): 4,
    ("B", "C"): 2,
    ("C", "B"): 2,
    ("B", "D"): 5,
    ("D", "B"): 5,
    ("C", "D"): 1,
    ("D", "C"): 1,
    ("C", "E"): 3,
    ("E", "C"): 3,
    ("D", "E"): 2,
    ("E", "D"): 2,
}


STATIC_ROUTING_TABLE = {
    "A": {"A": "A", "B": "B", "C": "B", "D": "C", "E": "C"},
    "B": {"A": "A", "B": "B", "C": "C", "D": "C", "E": "C"},
    "C": {"A": "B", "B": "B", "C": "C", "D": "D", "E": "E"},
    "D": {"A": "C", "B": "C", "C": "C", "D": "D", "E": "E"},
    "E": {"A": "C", "B": "C", "C": "C", "D": "D", "E": "E"},
}

NODES = ["A", "B", "C", "D", "E"]


def get_neighbors(node):
    """Get all neighbors of a node with their costs"""
    neighbors = []
    for (src, dst), cost in NETWORK_TOPOLOGY.items():
        if src == node:
            neighbors.append((dst, cost))
    return neighbors


def dijkstra(source, destination, failed_links=None):
    """
    Dijkstra's shortest path algorithm
    Returns: path, total_cost, steps_taken, nodes_explored
    """
    if failed_links is None:
        failed_links = set()

    distances = {node: float('inf') for node in NODES}
    distances[source] = 0
    previous = {node: None for node in NODES}
    pq = [(0, source)]
    visited = set()
    steps = []
    nodes_explored = []

    steps.append(f"Starting Dijkstra from {source} to {destination}")

    while pq:
        current_dist, current = heapq.heappop(pq)

        if current in visited:
            continue

        visited.add(current)
        nodes_explored.append(current)
        steps.append(f"Visiting node {current} (distance: {current_dist})")

        if current == destination:
            steps.append(f"Reached destination {destination}")
            break

        for neighbor, cost in get_neighbors(current):
            link = (current, neighbor)
            if link in failed_links:
                steps.append(f"Link {current}->{neighbor} is failed, skipping")
                continue

            distance = current_dist + cost

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current
                heapq.heappush(pq, (distance, neighbor))
                steps.append(
                    f"Updated distance to {neighbor}: {distance} via {current}")

    path = []
    current = destination
    while current is not None:
        path.append(current)
        current = previous[current]
    path.reverse()

    if path[0] != source:
        return None, float('inf'), steps, nodes_explored

    return path, distances[destination], steps, nodes_explored


def bellman_ford(source, destination, failed_links=None):
    """
    Bellman-Ford algorithm (handles negative weights, detects negative cycles)
    Returns: path, total_cost, steps_taken, iterations
    """
    if failed_links is None:
        failed_links = set()

    distances = {node: float('inf') for node in NODES}
    distances[source] = 0
    previous = {node: None for node in NODES}
    steps = []

    steps.append(f"Starting Bellman-Ford from {source} to {destination}")

    for iteration in range(len(NODES) - 1):
        updated = False
        steps.append(f"\nIteration {iteration + 1}:")

        for (u, v), cost in NETWORK_TOPOLOGY.items():
            if (u, v) in failed_links:
                continue

            if distances[u] + cost < distances[v]:
                distances[v] = distances[u] + cost
                previous[v] = u
                steps.append(
                    f"  Updated {v}: distance = {distances[v]} via {u}")
                updated = True

        if not updated:
            steps.append("  No updates, converged early")
            break

    for (u, v), cost in NETWORK_TOPOLOGY.items():
        if (u, v) in failed_links:
            continue
        if distances[u] + cost < distances[v]:
            steps.append("Negative cycle detected!")
            return None, float('inf'), steps, iteration + 1

    path = []
    current = destination
    while current is not None:
        path.append(current)
        current = previous[current]
    path.reverse()

    if path[0] != source:
        return None, float('inf'), steps, iteration + 1

    return path, distances[destination], steps, iteration + 1


def static_routing(source, destination, failed_links=None):
    """Use static routing table"""
    if failed_links is None:
        failed_links = set()

    path = [source]
    decisions = []
    visited = set()
    current = source
    total_cost = 0

    decisions.append(
        f"Using static routing table from {source} to {destination}")

    while current != destination:
        if current in visited:
            decisions.append(f"Loop detected at {current}")
            return None, float('inf'), decisions

        visited.add(current)
        next_hop = STATIC_ROUTING_TABLE[current].get(destination)

        if next_hop is None:
            decisions.append(f"No route to {destination} from {current}")
            return None, float('inf'), decisions

        if (current, next_hop) in failed_links:
            decisions.append(
                f"Link {current}->{next_hop} failed. Static routing cannot adapt.")
            return None, float('inf'), decisions

        cost = NETWORK_TOPOLOGY.get((current, next_hop), 1)
        total_cost += cost

        decisions.append(
            f"At {current}, routing table says next hop to {destination} is {next_hop} (cost: {cost})")
        path.append(next_hop)
        current = next_hop

        if len(path) > len(NODES) + 5:
            decisions.append("Path too long")
            return None, float('inf'), decisions

    decisions.append(
        f"Reached destination {destination}. Total cost: {total_cost}")
    return path, total_cost, decisions


def hop_count_routing(source, destination, failed_links=None):
    """Simple hop-count based routing (BFS)"""
    if failed_links is None:
        failed_links = set()

    from collections import deque

    queue = deque([(source, [source])])
    visited = {source}
    steps = []

    steps.append(
        f"Starting hop-count routing (BFS) from {source} to {destination}")

    while queue:
        current, path = queue.popleft()
        steps.append(f"Exploring {current}, current path: {' -> '.join(path)}")

        if current == destination:
            cost = len(path) - 1 
            steps.append(f"Found destination with {cost} hops")
            return path, cost, steps

        for neighbor, _ in get_neighbors(current):
            if (current, neighbor) in failed_links:
                steps.append(f"Link {current}->{neighbor} is failed, skipping")
                continue

            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))

    steps.append(f"No path found to {destination}")
    return None, float('inf'), steps


@app.route("/", methods=["GET", "POST"])
def index():
    path = None
    decisions = []
    src = dst = None
    if request.method == "POST":
        src = request.form.get("source")
        dst = request.form.get("destination")
        if src and dst:
            if src == dst:
                path = [src]
                decisions = [f"Source and destination are the same: {src}"]
            else:
                path, cost, decisions = static_routing(src, dst)

    topology_json = {}
    for (src_node, dst_node), cost in NETWORK_TOPOLOGY.items():
        key = f"{src_node},{dst_node}"
        topology_json[key] = cost

    return render_template(
        "index.html",
        nodes=NODES,
        path=path,
        decisions=decisions,
        src=src,
        dst=dst,
        routing_table=STATIC_ROUTING_TABLE,
        topology=topology_json
    )


@app.route("/compute", methods=["POST"])
def compute():
    data = request.json
    source = data.get("source")
    destination = data.get("destination")
    algorithm = data.get("algorithm", "dijkstra")
    failed_links_list = data.get("failed_links", [])

    failed_links = set()
    for link in failed_links_list:
        parts = link.split("-")
        if len(parts) == 2:
            failed_links.add((parts[0], parts[1]))

    if not source or not destination:
        return jsonify({"error": "Source and destination required"}), 400

    if source == destination:
        return jsonify({
            "path": [source],
            "cost": 0,
            "steps": [f"Source and destination are the same: {source}"],
            "metrics": {"nodes_explored": 1, "time_ms": 0}
        })

    start_time = time.time()

    if algorithm == "dijkstra":
        path, cost, steps, nodes_explored = dijkstra(
            source, destination, failed_links)
        metrics = {"nodes_explored": len(
            nodes_explored), "algorithm": "Dijkstra"}
    elif algorithm == "bellman_ford":
        path, cost, steps, iterations = bellman_ford(
            source, destination, failed_links)
        metrics = {"iterations": iterations, "algorithm": "Bellman-Ford"}
    elif algorithm == "static":
        path, cost, steps = static_routing(source, destination, failed_links)
        metrics = {"algorithm": "Static Routing"}
    elif algorithm == "hop_count":
        path, cost, steps = hop_count_routing(
            source, destination, failed_links)
        metrics = {"algorithm": "Hop Count (BFS)"}
    else:
        return jsonify({"error": "Invalid algorithm"}), 400

    end_time = time.time()
    execution_time = (end_time - start_time) * 1000 
    metrics["time_ms"] = round(execution_time, 4)

    return jsonify({
        "path": path,
        "cost": cost if cost != float('inf') else None,
        "steps": steps,
        "metrics": metrics,
        "success": path is not None
    })


@app.route("/compare", methods=["POST"])
def compare():
    """Compare all algorithms"""
    data = request.json
    source = data.get("source")
    destination = data.get("destination")
    failed_links_list = data.get("failed_links", [])

    failed_links = set()
    for link in failed_links_list:
        parts = link.split("-")
        if len(parts) == 2:
            failed_links.add((parts[0], parts[1]))

    results = {}

    start = time.time()
    path, cost, steps, nodes_explored = dijkstra(
        source, destination, failed_links)
    results["dijkstra"] = {
        "path": path,
        "cost": cost if cost != float('inf') else None,
        "time_ms": round((time.time() - start) * 1000, 4),
        "nodes_explored": len(nodes_explored)
    }

    start = time.time()
    path, cost, steps, iterations = bellman_ford(
        source, destination, failed_links)
    results["bellman_ford"] = {
        "path": path,
        "cost": cost if cost != float('inf') else None,
        "time_ms": round((time.time() - start) * 1000, 4),
        "iterations": iterations
    }

    start = time.time()
    path, cost, steps = static_routing(source, destination, failed_links)
    results["static"] = {
        "path": path,
        "cost": cost if cost != float('inf') else None,
        "time_ms": round((time.time() - start) * 1000, 4)
    }

    start = time.time()
    path, cost, steps = hop_count_routing(source, destination, failed_links)
    results["hop_count"] = {
        "path": path,
        "cost": cost if cost != float('inf') else None,
        "time_ms": round((time.time() - start) * 1000, 4)
    }

    return jsonify(results)


if __name__ == "__main__":
    app.run(debug=True)

