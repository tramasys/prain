const TILE_WIDTH = 40;
const GRID_SIZE = 10;
const EDGE_THICKNESS = 2; // Dicke der Kanten in Pixel

document.addEventListener("DOMContentLoaded", async () => {
  const grid = document.getElementById("grid");
  grid.style.gridTemplateColumns = `repeat(${GRID_SIZE}, ${TILE_WIDTH}px)`;
  grid.style.gridTemplateRows = `repeat(${GRID_SIZE}, ${TILE_WIDTH}px)`;

  // Erstellt das Gitter visuell von oben (y=9) nach unten (y=0)
  for (let y = GRID_SIZE - 1; y >= 0; y--) {
    for (let x = 0; x < GRID_SIZE; x++) {
      const tile = document.createElement("div");
      tile.className = "tile";
      tile.dataset.x = x;
      tile.dataset.y = y;
      tile.style.width = `${TILE_WIDTH}px`;
      tile.style.height = `${TILE_WIDTH}px`;

      if (x === 0) {
        tile.classList.add("y-label");
      }
      if (y === 0) {
        tile.classList.add("x-label");
      }

      tile.addEventListener("click", () => handleTileClick(x, y));
      grid.appendChild(tile);
    }
  }

  // Wichtig: ?t=... verhindert Caching-Probleme
  const graph = await fetch(`/get_graph?t=${new Date().getTime()}`).then((r) =>
    r.json()
  );
  drawNodes(graph.nodes);
  drawEdges(graph.edges, graph.nodes);

  document.getElementById("clear-btn").addEventListener("click", () => {
    if (confirm("Are you sure you want to clear the entire graph?")) {
      fetch("/clear", { method: "POST" }).then(() => location.reload());
    }
  });
});

function handleTileClick(x, y) {
  const label = prompt(`Enter node label at (${x},${y}):`);
  if (!label) return;

  fetch("/add_node", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ label, x, y }),
  }).then(() => location.reload());
}

function drawNodes(nodes) {
  for (const [label, { x, y }] of Object.entries(nodes)) {
    const tile = document.querySelector(`.tile[data-x='${x}'][data-y='${y}']`);
    if (tile) {
      tile.classList.add("node");
      tile.innerText = label;
      tile.title = `Position: (${x}, ${y})`;
    } else {
      console.error(`Could not find tile for node ${label} at (${x},${y})`);
    }
  }
}

function drawEdges(edges, nodes) {
  const overlay = document.getElementById("edge-overlay");
  overlay.innerHTML = "";
  const drawnEdges = new Set();

  edges.forEach(({ from, to }) => {
    const edgeKey = [from, to].sort().join("-");
    if (drawnEdges.has(edgeKey)) {
      return;
    }
    drawnEdges.add(edgeKey);

    const a = nodes[from];
    const b = nodes[to];
    if (!a || !b) return;

    const x1 = a.x * TILE_WIDTH + TILE_WIDTH / 2;
    const y1 = (GRID_SIZE - 1 - a.y) * TILE_WIDTH + TILE_WIDTH / 2;
    const x2 = b.x * TILE_WIDTH + TILE_WIDTH / 2;
    const y2 = (GRID_SIZE - 1 - b.y) * TILE_WIDTH + TILE_WIDTH / 2;

    const dx = x2 - x1;
    const dy = y2 - y1;
    const length = Math.sqrt(dx * dx + dy * dy);
    const angle = Math.atan2(dy, dx) * (180 / Math.PI);

    const edge = document.createElement("div");
    edge.className = "edge";
    edge.style.width = `${length}px`;
    edge.style.transform = `translate(${x1}px, ${y1}px) rotate(${angle}deg) translateY(-${
      EDGE_THICKNESS / 2
    }px)`;

    overlay.appendChild(edge);
  });
}
