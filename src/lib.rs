use petgraph::{
    stable_graph::{NodeIndex, StableUnGraph},
    visit::{EdgeRef, IntoEdgeReferences},
};

pub type DefaultNodeIdx = NodeIndex<petgraph::stable_graph::DefaultIx>;

const FORCE_CHARGE: f32 = 12000.0;
const FORCE_SPRING: f32 = 0.3;

const FORCE_MAX: f32 = 280.0;
const NODE_SPEED: f32 = 7000.0;
const DAMPING_FACTOR: f32 = 0.95;

pub struct Node<D> {
    pub x: f32,
    pub y: f32,
    vx: f32,
    vy: f32,
    ax: f32,
    ay: f32,
    mass: f32,
    is_anchor: bool,
    pub data: D,
}

impl<D> Node<D> {
    fn apply_force(&mut self, fx: f32, fy: f32, dt: f32) {
        self.ax += fx.max(-FORCE_MAX).min(FORCE_MAX) * dt;
        self.ay += fy.max(-FORCE_MAX).min(FORCE_MAX) * dt;
    }

    fn update(&mut self, dt: f32) {
        self.vx = (self.vx + self.ax * dt * NODE_SPEED) * DAMPING_FACTOR;
        self.vy = (self.vy + self.ay * dt * NODE_SPEED) * DAMPING_FACTOR;
        self.x += self.vx * dt;
        self.y += self.vy * dt;
        self.ax = 0.0;
        self.ay = 0.0;
    }
}

/// W for edge weight. If you don't need any data on your
/// links use the empty tuple for W: ().
pub struct Edge<W> {
    pub weight: W,
}

pub struct ForceGraph<D, W> {
    graph: StableUnGraph<Node<D>, Edge<W>>,
}

impl<D, W> ForceGraph<D, W> {
    pub fn new() -> Self {
        ForceGraph {
            graph: StableUnGraph::default(),
        }
    }

    pub fn get_graph(&self) -> &StableUnGraph<Node<D>, Edge<W>> {
        &self.graph
    }

    pub fn add_node(
        &mut self,
        x: f32,
        y: f32,
        data: D,
        mass: f32,
        is_anchor: bool,
    ) -> DefaultNodeIdx {
        self.graph.add_node(Node {
            x,
            y,
            vx: 0.0,
            vy: 0.0,
            ax: 0.0,
            ay: 0.0,
            mass,
            is_anchor,
            data,
        })
    }

    pub fn remove_node(&mut self, idx: DefaultNodeIdx) {
        self.graph.remove_node(idx);
    }

    pub fn add_edge(&mut self, n1_idx: DefaultNodeIdx, n2_idx: DefaultNodeIdx, edge: Edge<W>) {
        self.graph.update_edge(n1_idx, n2_idx, edge);
    }

    pub fn clear(&mut self) {
        self.graph.clear();
    }

    pub fn update(&mut self, dt: f32) {
        if self.graph.node_count() == 0 {
            return;
        }

        let first_node_idx = self
            .graph
            .node_indices()
            .next()
            .expect("first node missing");

        let mut bfs = petgraph::visit::Bfs::new(&self.graph, first_node_idx);

        while let Some(n1) = bfs.next(&self.graph) {
            let mut edges = self.graph.neighbors(n1).detach();
            while let Some(n2) = edges.next_node(&self.graph) {
                let f = attract_nodes(&self.graph[n1], &self.graph[n2]);
                self.graph[n1].apply_force(f.0, f.1, dt);
            }
        }

        let mut bfs1 = petgraph::visit::Bfs::new(&self.graph, first_node_idx);

        while let Some(n1) = bfs1.next(&self.graph) {
            if self.graph[n1].is_anchor {
                continue;
            }

            let mut bfs2 = petgraph::visit::Bfs::new(&self.graph, n1);
            bfs2.next(&self.graph);

            while let Some(n2) = bfs2.next(&self.graph) {
                let f = repel_nodes(&self.graph[n1], &self.graph[n2]);
                self.graph[n1].apply_force(f.0, f.1, dt);
            }

            self.graph[n1].update(dt);
        }
    }

    pub fn visit_nodes<F: FnMut(&Node<D>)>(&self, mut cb: F) {
        for n_idx in self.graph.node_indices() {
            cb(&self.graph[n_idx]);
        }
    }

    pub fn visit_edges<F: FnMut(&Node<D>, &Node<D>, &Edge<W>)>(&self, mut cb: F) {
        for edge_ref in self.graph.edge_references() {
            let source = &self.graph[edge_ref.source()];
            let target = &self.graph[edge_ref.target()];
            let weight = edge_ref.weight();
            cb(source, target, weight);
        }
    }
}

fn attract_nodes<D>(n1: &Node<D>, n2: &Node<D>) -> (f32, f32) {
    let mut dx = n2.x - n1.x;
    let mut dy = n2.y - n1.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let strength = 1.0 * FORCE_SPRING * distance * 0.5;
    (dx * strength, dy * strength)
}

fn repel_nodes<D>(n1: &Node<D>, n2: &Node<D>) -> (f32, f32) {
    let mut dx = n2.x - n1.x;
    let mut dy = n2.y - n1.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let strength = -FORCE_CHARGE * ((n1.mass * n2.mass) / (distance * distance));
    (dx * strength, dy * strength)
}
