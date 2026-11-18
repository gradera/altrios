//! Standalone track network and routing crate.
//!
//! Provides a [`Network`] data model (compatible with the ALTRIOS `Link`
//! schema) and shortest-path [`shortest_path`] routing between links.

pub mod network;
pub mod router;

pub use network::{Elev, Heading, Link, LinkIdx, Location, Network, SpeedLimit, SpeedSet};
pub use router::{reachable, route, shortest_path, validate_od_pairs, Route, RoutePolicy};
