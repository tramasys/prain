from flask import Flask, render_template, request, jsonify
import json
import os

app = Flask(__name__)

GRID_SIZE = 10
TILE_WIDTH = 40  # Pixel size of each tile
DATA_FILE = 'src/brain/graph_config/graph_data.json'

# Initialize graph data structure
graph = {
    "nodes": {},  # e.g., "S": {"x": 0, "y": 0}
    "edges": [
        {"from": "S", "to": "1"}, {"from": "1", "to": "S"},
        {"from": "S", "to": "2"}, {"from": "2", "to": "S"},
        {"from": "S", "to": "3"}, {"from": "3", "to": "S"},
        {"from": "1", "to": "2"}, {"from": "2", "to": "1"},
        {"from": "2", "to": "3"}, {"from": "3", "to": "2"},
        {"from": "1", "to": "A"}, {"from": "A", "to": "1"},
        {"from": "2", "to": "A"}, {"from": "A", "to": "2"},
        {"from": "2", "to": "4"}, {"from": "4", "to": "2"},
        {"from": "3", "to": "4"}, {"from": "4", "to": "3"},
        {"from": "3", "to": "C"}, {"from": "C", "to": "3"},
        {"from": "A", "to": "4"}, {"from": "4", "to": "A"},
        {"from": "4", "to": "B"}, {"from": "B", "to": "4"},
        {"from": "B", "to": "C"}, {"from": "C", "to": "B"},
        {"from": "4", "to": "C"}, {"from": "C", "to": "4"},
        {"from": "A", "to": "B"}, {"from": "B", "to": "A"}
    ]   # e.g., [{"from": "S", "to": "1"}]
}

# Load graph from file if it exists
if os.path.exists(DATA_FILE):
    with open(DATA_FILE, 'r') as f:
        loaded = json.load(f)
        graph['nodes'] = loaded.get('nodes', {})
        graph['edges'] = loaded['edges'] if loaded.get('edges') else graph['edges']
        print(graph)

@app.route('/')
def index():
    return render_template('index.html', grid_size=GRID_SIZE, tile_width=TILE_WIDTH, graph=graph)

@app.route('/add_node', methods=['POST'])
def add_node():
    data = request.json
    label = data['label']
    x, y = data['x'], data['y']
    graph['nodes'][label] = {"x": x, "y": y}
    _save_graph()
    return jsonify(success=True)

@app.route('/get_graph')
def get_graph():
    return jsonify(graph)

@app.route('/clear', methods=['POST'])
def clear_graph():
    graph['nodes'].clear()
    _save_graph()
    return jsonify(success=True)


def _save_graph():
    with open(DATA_FILE, 'w') as f:
        json.dump(graph, f, indent=2)

if __name__ == '__main__':
    app.run(debug=True)
