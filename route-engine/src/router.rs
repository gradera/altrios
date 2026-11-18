//! Shortest-path routing over a [`Network`].

use crate::network::{Link, LinkIdx, Network};
use anyhow::{bail, Result};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

/// A resolved route between two links.
#[derive(Debug, Clone, PartialEq)]
pub struct Route {
    /// Ordered link indices from origin to destination.
    pub link_path: Vec<LinkIdx>,
    /// Total route length (m).
    pub total_length: f64,
}

/// Path-selection policy. Determines the cost assigned to each link during the
/// search. All policies resolve the path once, up front.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RoutePolicy {
    /// Minimize total distance (link length).
    Shortest,
    /// Minimize distance plus a penalty on absolute elevation change (avoids
    /// steep climbs). `penalty` is meters-of-length equivalent per meter of
    /// elevation change.
    LeastGrade {
        /// Penalty factor applied to absolute elevation change.
        penalty: f64,
    },
}

impl Default for RoutePolicy {
    fn default() -> Self {
        RoutePolicy::Shortest
    }
}

impl RoutePolicy {
    /// Cost of traversing `link` under this policy.
    fn link_cost(&self, link: &Link) -> f64 {
        match self {
            RoutePolicy::Shortest => link.length,
            RoutePolicy::LeastGrade { penalty } => {
                let dh = link
                    .elevs
                    .last()
                    .zip(link.elevs.first())
                    .map(|(e1, e0)| (e1.elev - e0.elev).abs())
                    .unwrap_or(0.0);
                link.length + penalty * dh
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: f64,
    node: LinkIdx,
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for a min-heap.
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Resolve a route from `origin` to `destination` under the given policy.
///
/// The path is resolved once, before the tick loop; it is static for the run.
pub fn route(
    network: &Network,
    origin: LinkIdx,
    destination: LinkIdx,
    policy: RoutePolicy,
) -> Result<Route> {
    if network.link(origin).is_none() {
        bail!("origin link {:?} not found in network", origin);
    }
    if network.link(destination).is_none() {
        bail!("destination link {:?} not found in network", destination);
    }

    let mut dist: HashMap<LinkIdx, f64> = HashMap::new();
    let mut prev: HashMap<LinkIdx, LinkIdx> = HashMap::new();
    let mut heap = BinaryHeap::new();

    dist.insert(origin, 0.0);
    heap.push(State {
        cost: 0.0,
        node: origin,
    });

    while let Some(State { cost, node }) = heap.pop() {
        if node == destination {
            break;
        }
        if cost > *dist.get(&node).unwrap_or(&f64::INFINITY) {
            continue;
        }
        let link = match network.link(node) {
            Some(l) => l,
            None => continue,
        };
        for next in link.next_links() {
            let next_link = match network.link(next) {
                Some(l) => l,
                None => continue,
            };
            let next_cost = cost + policy.link_cost(next_link);
            if next_cost < *dist.get(&next).unwrap_or(&f64::INFINITY) {
                dist.insert(next, next_cost);
                prev.insert(next, node);
                heap.push(State {
                    cost: next_cost,
                    node: next,
                });
            }
        }
    }

    if !dist.contains_key(&destination) {
        bail!("no path found from {:?} to {:?}", origin, destination);
    }

    // Reconstruct path.
    let mut path = vec![destination];
    let mut cur = destination;
    while cur != origin {
        match prev.get(&cur) {
            Some(&p) => {
                path.push(p);
                cur = p;
            }
            None => bail!("path reconstruction failed at {:?}", cur),
        }
    }
    path.reverse();

    let total_length = path
        .iter()
        .filter_map(|i| network.link(*i))
        .map(|l| l.length)
        .sum();

    Ok(Route {
        link_path: path,
        total_length,
    })
}

/// Resolve the shortest path (by length) from `origin` to `destination`.
///
/// Convenience wrapper around [`route`] with [`RoutePolicy::Shortest`].
pub fn shortest_path(network: &Network, origin: LinkIdx, destination: LinkIdx) -> Result<Route> {
    route(network, origin, destination, RoutePolicy::Shortest)
}

/// Whether `destination` is reachable from `origin`.
pub fn reachable(network: &Network, origin: LinkIdx, destination: LinkIdx) -> bool {
    route(network, origin, destination, RoutePolicy::Shortest).is_ok()
}

/// Validate that at least one origin/destination pair is connected.
///
/// Returns the list of `(origin, destination)` pairs that are connected, or an
/// error if none are. Mirrors ALTRIOS's OD-pair validity check (but at the
/// resolved-path level rather than the link-option level).
pub fn validate_od_pairs(
    network: &Network,
    origins: &[LinkIdx],
    destinations: &[LinkIdx],
) -> Result<Vec<(LinkIdx, LinkIdx)>> {
    let mut valid = Vec::new();
    for &o in origins {
        for &d in destinations {
            if reachable(network, o, d) {
                valid.push((o, d));
            }
        }
    }
    if valid.is_empty() {
        bail!(
            "no valid origin/destination pairs: origins {:?}, destinations {:?}",
            origins,
            destinations
        );
    }
    Ok(valid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::Link;

    fn link(idx: u32, next: u32, next_alt: u32, length: f64) -> Link {
        Link {
            idx_curr: LinkIdx(idx),
            idx_next: LinkIdx(next),
            idx_next_alt: LinkIdx(next_alt),
            length,
            ..Default::default()
        }
    }

    #[test]
    fn shortest_path_branch() {
        // 1 -> 2 -> 4 (short) ; 1 -> 3 -> 4 (long)
        let links = vec![
            Link::default(), // sentinel
            link(1, 2, 3, 100.0),
            link(2, 4, 0, 100.0),
            link(3, 4, 0, 500.0),
            link(4, 0, 0, 100.0),
        ];
        let net = Network::new(links);
        let route = shortest_path(&net, LinkIdx(1), LinkIdx(4)).unwrap();
        assert_eq!(route.link_path, vec![LinkIdx(1), LinkIdx(2), LinkIdx(4)]);
        assert!((route.total_length - 300.0).abs() < 1e-9);
    }

    #[test]
    fn reachability() {
        let links = vec![
            Link::default(),
            link(1, 2, 0, 100.0),
            link(2, 0, 0, 100.0),
            // Link 3 is isolated.
            link(3, 0, 0, 100.0),
        ];
        let net = Network::new(links);
        assert!(reachable(&net, LinkIdx(1), LinkIdx(2)));
        assert!(!reachable(&net, LinkIdx(1), LinkIdx(3)));
        assert!(!reachable(&net, LinkIdx(2), LinkIdx(1))); // one-way
    }

    #[test]
    fn od_pair_validation() {
        let links = vec![
            Link::default(),
            link(1, 2, 0, 100.0),
            link(2, 0, 0, 100.0),
            link(3, 0, 0, 100.0),
        ];
        let net = Network::new(links);
        // (1,2) works; (3,2) and (3,1) don't.
        let valid =
            validate_od_pairs(&net, &[LinkIdx(1), LinkIdx(3)], &[LinkIdx(2)]).unwrap();
        assert_eq!(valid, vec![(LinkIdx(1), LinkIdx(2))]);
        // No valid pairs at all -> error.
        assert!(validate_od_pairs(&net, &[LinkIdx(3)], &[LinkIdx(1)]).is_err());
    }

    #[test]
    fn least_grade_policy_avoids_steep_route() {
        // Two routes 1->3: link 2 is short but climbs, link 4 is longer but flat.
        let mut flat = link(4, 3, 0, 300.0);
        flat.elevs = vec![
            crate::network::Elev { offset: 0.0, elev: 0.0 },
            crate::network::Elev { offset: 300.0, elev: 0.0 },
        ];
        let mut steep = link(2, 3, 0, 150.0);
        steep.elevs = vec![
            crate::network::Elev { offset: 0.0, elev: 0.0 },
            crate::network::Elev { offset: 150.0, elev: 150.0 },
        ];
        let links = vec![
            Link::default(),
            link(1, 2, 4, 100.0),
            steep,
            link(3, 0, 0, 100.0),
            flat,
        ];
        let net = Network::new(links);

        // Shortest picks the short steep route (1,2,3).
        let r_short = route(&net, LinkIdx(1), LinkIdx(3), RoutePolicy::Shortest).unwrap();
        assert_eq!(r_short.link_path, vec![LinkIdx(1), LinkIdx(2), LinkIdx(3)]);

        // LeastGrade with a big penalty avoids the steep link (1,4,3).
        let r_grade = route(
            &net,
            LinkIdx(1),
            LinkIdx(3),
            RoutePolicy::LeastGrade { penalty: 5.0 },
        )
        .unwrap();
        assert_eq!(r_grade.link_path, vec![LinkIdx(1), LinkIdx(4), LinkIdx(3)]);
    }
}
