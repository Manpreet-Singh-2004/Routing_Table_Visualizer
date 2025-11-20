# Interactive Routing Visualizer

This is a simple web-based application built with Python (Flask) and JavaScript that visualizes the pathfinding of several common routing algorithms on a static network topology.

Users can select a source, a destination, and an algorithm, and the application will display the computed path, its cost, and a step-by-step log of how the algorithm found the route.

# Features

- Visual Network Topology: An interactive canvas displays the 5-node network (A-E) and their link costs.

- Algorithm Selection: Implements four different routing algorithms:

  - Static Routing: Uses a pre-computed routing table.

  - Dijkstra: Finds the shortest (lowest-cost) path.

  - Bellman-Ford: Finds the shortest path, capable of handling negative weights.

  - Hop Count (BFS): Finds the path with the fewest hops, ignoring cost.

- Dynamic Results: The UI instantly updates with:

  - The full path (e.g., A → C → D).

  - The total cost of the path.

  - A step-by-step log of the algorithm's decision-making process.

- Path Animation: An animated "packet" travels along the computed path on the network map.

# How to Run the Solution

## Prerequisites

- Python 3

- pip (Python package installer)

## Installation & Setup

1. Clone or download this repository.
  You should have a folder containing app.py, a templates folder (with index.html), and a static folder.

2. Navigate to the project directory:

  ```bash
  cd path/to/your/project/folder
  ```

3. (Optional but Recommended) Create a virtual environment:

  ```bash
  python -m venv venv
  source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
  ```

4. Install the only requirement (Flask):

  ```bash
  pip install Flask
  ```

5. Running the Application

  Run the Flask server:
  
  ```bash
  python app.py
  ```

Open the application in your browser:
Navigate to http://127.0.0.1:5000 in your web browser.

# Input / Output Format

## Input (User UI):

- Source Router: Select from dropdown (A, B, C, D, E).

- Destination Router: Select from dropdown (A, B, C, D, E).

- Routing Algorithm: Select from dropdown (Static, Dijkstra, Bellman-Ford, Hop Count).

## Output (Application UI):

- Path: A string showing the nodes in order (e.g., "A → B → C").

- Cost: A number representing the total cost of the path.

- Steps: A list of strings explaining each step the algorithm took.

- Visual: The computed path is highlighted on the canvas
