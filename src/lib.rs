//! A Rust implementation of the force-directed graph algorithm from [Graphoon](https://github.com/rm-code/Graphoon/).
//!
//! # Example
//!
//! ```
//! use force_graph::{ForceGraph, Node, NodeData};
//!
//! // create a force graph with default parameters
//! let mut graph = <ForceGraph>::new(Default::default());
//!
//! // create nodes
//! let n1_idx = graph.add_node(NodeData {
//!     x: 250.0,
//!     y: 250.0,
//!     ..Default::default()
//! });
//! let n2_idx = graph.add_node(NodeData {
//!     x: 750.0,
//!     y: 250.0,
//!     ..Default::default()
//! });
//! let n3_idx = graph.add_node(NodeData {
//!     x: 250.0,
//!     y: 750.0,
//!     ..Default::default()
//! });
//! let n4_idx = graph.add_node(NodeData {
//!     x: 750.0,
//!     y: 750.0,
//!     ..Default::default()
//! });
//! let n5_idx = graph.add_node(NodeData {
//!     x: 500.0,
//!     y: 500.0,
//!     is_anchor: true,
//!     ..Default::default()
//! });
//!
//! // set up links between nodes
//! graph.add_edge(n1_idx, n5_idx, Default::default());
//! graph.add_edge(n2_idx, n5_idx, Default::default());
//! graph.add_edge(n3_idx, n5_idx, Default::default());
//! graph.add_edge(n4_idx, n5_idx, Default::default());
//!
//! // --- your game loop would start here ---
//!
//! // draw edges with your own drawing function
//! fn draw_edge(x1: f32, y1: f32, x2: f32, y2: f32) {}
//!
//! graph.visit_edges(|node1, node2, _edge| {
//!     draw_edge(node1.x(), node1.y(), node2.x(), node2.y());
//! });
//!
//! // draw nodes with your own drawing function
//! fn draw_node(x: f32, y: f32) {}
//!
//! graph.visit_nodes(|node| {
//!     draw_node(node.x(), node.y());
//! });
//!
//! // calculate dt with your own timing function
//! let dt = 0.1;
//! graph.update(dt);
//!
//! // --- your game loop would repeat here ---
//!
//! ```

use petgraph::{
    stable_graph::{NodeIndex, StableUnGraph},
    visit::{EdgeRef, IntoEdgeReferences},
};

use std::collections::BTreeSet;

pub type DefaultNodeIdx = NodeIndex<petgraph::stable_graph::DefaultIx>;

/// Parameters to control the simulation of the force graph.
#[derive(Clone, Debug)]
pub struct SimulationParameters {
    pub force_charge: f32,
    pub force_spring: f32,
    pub force_max: f32,
    pub node_speed: f32,
    pub damping_factor: f32,
}

impl Default for SimulationParameters {
    fn default() -> Self {
        SimulationParameters {
            force_charge: 12000.0,
            force_spring: 0.3,
            force_max: 280.0,
            node_speed: 7000.0,
            damping_factor: 0.95,
        }
    }
}

/// Stores data associated with a node that can be modified by the user.
pub struct NodeData<UserNodeData = ()> {
    /// The horizontal position of the node.
    pub x: f32,
    /// The vertical position of the node.
    pub y: f32,
    /// The mass of the node.
    ///
    /// Increasing the mass of a node increases the force with which it repels other nearby nodes.
    pub mass: f32,
    /// Whether the node is fixed to its current position.
    pub is_anchor: bool,
    /// Arbitrary user data.
    ///
    /// Defaults to `()` if not specified.
    pub user_data: UserNodeData,
}

impl<UserNodeData> Default for NodeData<UserNodeData>
where
    UserNodeData: Default,
{
    fn default() -> Self {
        NodeData {
            x: 0.0,
            y: 0.0,
            mass: 10.0,
            is_anchor: false,
            user_data: Default::default(),
        }
    }
}

/// Stores data associated with an edge that can be modified by the user.
pub struct EdgeData<UserEdgeData = ()> {
    /// Arbitrary user data.
    ///
    /// Defaults to `()` if not specified.
    pub user_data: UserEdgeData,
}

impl<UserEdgeData> Default for EdgeData<UserEdgeData>
where
    UserEdgeData: Default,
{
    fn default() -> Self {
        EdgeData {
            user_data: Default::default(),
        }
    }
}

/// The main force graph structure.
pub struct ForceGraph<UserNodeData = (), UserEdgeData = ()> {
    pub parameters: SimulationParameters,
    graph: StableUnGraph<Node<UserNodeData>, EdgeData<UserEdgeData>>,
    node_indices: BTreeSet<DefaultNodeIdx>,
}

impl<UserNodeData, UserEdgeData> ForceGraph<UserNodeData, UserEdgeData> {
    /// Constructs a new force graph.
    ///
    /// Use the following syntax to create a graph with default parameters:
    /// ```
    /// use force_graph::ForceGraph;
    /// let graph = <ForceGraph>::new(Default::default());
    /// ```
    pub fn new(parameters: SimulationParameters) -> Self {
        ForceGraph {
            parameters,
            graph: StableUnGraph::default(),
            node_indices: Default::default(),
        }
    }

    /// Provides access to the raw graph structure if required.
    pub fn get_graph(&self) -> &StableUnGraph<Node<UserNodeData>, EdgeData<UserEdgeData>> {
        &self.graph
    }

    /// Adds a new node and returns an index that can be used to reference the node.
    pub fn add_node(&mut self, node_data: NodeData<UserNodeData>) -> DefaultNodeIdx {
        let idx = self.graph.add_node(Node {
            data: node_data,
            index: Default::default(),
            vx: 0.0,
            vy: 0.0,
            ax: 0.0,
            ay: 0.0,
        });
        self.graph[idx].index = idx;
        self.node_indices.insert(idx);
        idx
    }

    /// Removes a node by index.
    pub fn remove_node(&mut self, idx: DefaultNodeIdx) {
        self.graph.remove_node(idx);
        self.node_indices.remove(&idx);
    }

    /// Adds or updates an edge connecting two nodes by index.
    pub fn add_edge(
        &mut self,
        n1_idx: DefaultNodeIdx,
        n2_idx: DefaultNodeIdx,
        edge: EdgeData<UserEdgeData>,
    ) {
        self.graph.update_edge(n1_idx, n2_idx, edge);
    }

    /// Removes all nodes from the force graph.
    pub fn clear(&mut self) {
        self.graph.clear();
        self.node_indices.clear();
    }

    /// Applies the next step of the force graph simulation.
    ///
    /// The number of seconds that have elapsed since the previous update must be calculated and
    /// provided by the user as `dt`.
    pub fn update(&mut self, dt: f32) {
        if self.graph.node_count() == 0 {
            return;
        }

        for n1_idx in &self.node_indices {
            let mut edges = self.graph.neighbors(*n1_idx).detach();
            while let Some(n2_idx) = edges.next_node(&self.graph) {
                let f = attract_nodes(&self.graph[*n1_idx], &self.graph[n2_idx], &self.parameters);
                self.graph[*n1_idx].apply_force(f.0, f.1, dt, &self.parameters);
            }

            if self.graph[*n1_idx].data.is_anchor {
                continue;
            }

            for n2_idx in &self.node_indices {
                if n1_idx == n2_idx {
                    continue;
                }
                let f = repel_nodes(&self.graph[*n1_idx], &self.graph[*n2_idx], &self.parameters);
                self.graph[*n1_idx].apply_force(f.0, f.1, dt, &self.parameters);
            }

            self.graph[*n1_idx].update(dt, &self.parameters);
        }
    }

    /// Processes each node with a user-defined callback `cb`.
    pub fn visit_nodes<F: FnMut(&Node<UserNodeData>)>(&self, mut cb: F) {
        for n_idx in self.graph.node_indices() {
            cb(&self.graph[n_idx]);
        }
    }

    /// Mutates each node with a user-defined callback `cb`.
    pub fn visit_nodes_mut<F: FnMut(&mut Node<UserNodeData>)>(&mut self, mut cb: F) {
        for node in self.graph.node_weights_mut() {
            cb(node);
        }
    }

    /// Processes each edge and its associated nodes with a user-defined callback `cb`.
    pub fn visit_edges<
        F: FnMut(&Node<UserNodeData>, &Node<UserNodeData>, &EdgeData<UserEdgeData>),
    >(
        &self,
        mut cb: F,
    ) {
        for edge_ref in self.graph.edge_references() {
            let source = &self.graph[edge_ref.source()];
            let target = &self.graph[edge_ref.target()];
            let edge_data = edge_ref.weight();
            cb(source, target, edge_data);
        }
    }
}

/// References a node in the [ForceGraph]. Can not be constructed by the user.
pub struct Node<UserNodeData = ()> {
    /// The node data provided by the user.
    pub data: NodeData<UserNodeData>,
    index: DefaultNodeIdx,
    vx: f32,
    vy: f32,
    ax: f32,
    ay: f32,
}

impl<UserNodeData> Node<UserNodeData> {
    /// The horizontal position of the node.
    pub fn x(&self) -> f32 {
        self.data.x
    }

    /// The vertical position of the node.
    pub fn y(&self) -> f32 {
        self.data.y
    }

    /// The index used to reference the node in the [ForceGraph].
    pub fn index(&self) -> DefaultNodeIdx {
        self.index
    }

    fn apply_force(&mut self, fx: f32, fy: f32, dt: f32, parameters: &SimulationParameters) {
        self.ax += fx.max(-parameters.force_max).min(parameters.force_max) * dt;
        self.ay += fy.max(-parameters.force_max).min(parameters.force_max) * dt;
    }

    fn update(&mut self, dt: f32, parameters: &SimulationParameters) {
        self.vx = (self.vx + self.ax * dt * parameters.node_speed) * parameters.damping_factor;
        self.vy = (self.vy + self.ay * dt * parameters.node_speed) * parameters.damping_factor;
        self.data.x += self.vx * dt;
        self.data.y += self.vy * dt;
        self.ax = 0.0;
        self.ay = 0.0;
    }
}

fn attract_nodes<D>(n1: &Node<D>, n2: &Node<D>, parameters: &SimulationParameters) -> (f32, f32) {
    let mut dx = n2.data.x - n1.data.x;
    let mut dy = n2.data.y - n1.data.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let strength = 1.0 * parameters.force_spring * distance * 0.5;
    (dx * strength, dy * strength)
}

fn repel_nodes<D>(n1: &Node<D>, n2: &Node<D>, parameters: &SimulationParameters) -> (f32, f32) {
    let mut dx = n2.data.x - n1.data.x;
    let mut dy = n2.data.y - n1.data.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let distance_sqrd = distance * distance;
    let strength = -parameters.force_charge * ((n1.data.mass * n2.data.mass) / distance_sqrd);
    (dx * strength, dy * strength)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_default() {
        let mut graph = <ForceGraph>::new(Default::default());
        let n1_idx = graph.add_node(NodeData {
            x: 0.1,
            y: 0.2,
            ..Default::default()
        });
        let n2_idx = graph.add_node(NodeData {
            x: 0.3,
            y: 0.4,
            ..Default::default()
        });
        graph.add_edge(n1_idx, n2_idx, Default::default());
    }

    #[test]
    fn test_user_data() {
        let mut graph = ForceGraph::new(Default::default());

        #[derive(Default)]
        struct UserNodeData {}
        #[derive(Default)]
        struct UserEdgeData {}

        let n1_idx = graph.add_node(NodeData {
            x: 0.1,
            y: 0.2,
            user_data: UserNodeData {},
            ..Default::default()
        });
        let n2_idx = graph.add_node(NodeData {
            x: 0.3,
            y: 0.4,
            user_data: UserNodeData {},
            ..Default::default()
        });

        graph.add_edge(
            n1_idx,
            n2_idx,
            EdgeData {
                user_data: UserEdgeData {},
            },
        );
    }
}
