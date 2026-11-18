//! Track network data model.
//!
//! Mirrors the ALTRIOS `Link` schema so existing network files can be loaded,
//! while remaining fully independent of `altrios-core`.

use anyhow::{bail, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// Meters in one mile, for milepost/offset conversion.
pub const METERS_PER_MILE: f64 = 1609.344;

/// Index of a link in a [`Network`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, Default)]
pub struct LinkIdx(pub u32);

impl LinkIdx {
    /// Create a link index.
    pub fn new(idx: u32) -> Self {
        Self(idx)
    }
    /// Whether this is the sentinel "not available" index (0).
    pub fn is_fake(&self) -> bool {
        self.0 == 0
    }
    /// Raw index.
    pub fn idx(&self) -> usize {
        self.0 as usize
    }
}

/// An elevation point along a link.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Elev {
    /// Offset from link start (m).
    pub offset: f64,
    /// Elevation (m).
    pub elev: f64,
}

/// A heading point along a link.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Heading {
    /// Offset from link start (m).
    pub offset: f64,
    /// Heading (radians).
    pub heading: f64,
}

/// A speed limit segment along a link.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct SpeedLimit {
    /// Start offset (m).
    pub offset_start: f64,
    /// End offset (m).
    pub end_offset: f64,
    /// Speed limit (m/s).
    pub speed: f64,
}

/// A speed set for a train type.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct SpeedSet {
    /// Speed limits.
    #[serde(default)]
    pub speed_limits: Vec<SpeedLimit>,
    /// Whether the limits apply by the head end of the train. If `false`, each
    /// limit is extended by the train length so it stays in effect until the
    /// tail clears it.
    #[serde(default)]
    pub is_head_end: bool,
}

/// A unit of single track between junctions.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Link {
    /// This link's index.
    pub idx_curr: LinkIdx,
    /// Reverse-direction index.
    #[serde(default)]
    pub idx_flip: LinkIdx,
    /// Forward neighbor.
    #[serde(default)]
    pub idx_next: LinkIdx,
    /// Alternate forward neighbor (branch).
    #[serde(default)]
    pub idx_next_alt: LinkIdx,
    /// Backward neighbor.
    #[serde(default)]
    pub idx_prev: LinkIdx,
    /// Alternate backward neighbor (branch).
    #[serde(default)]
    pub idx_prev_alt: LinkIdx,
    /// Link length (m).
    pub length: f64,
    /// Elevation profile.
    #[serde(default)]
    pub elevs: Vec<Elev>,
    /// Heading profile.
    #[serde(default)]
    pub headings: Vec<Heading>,
    /// Speed sets keyed by train type name.
    #[serde(default)]
    pub speed_sets: HashMap<String, SpeedSet>,
    /// Optional train-type-neutral speed set.
    #[serde(default)]
    pub speed_set: Option<SpeedSet>,
    /// Subdivision (region) this link belongs to.
    #[serde(default)]
    pub subdivision: Option<String>,
    /// Specific track name (e.g., "Main 1", "Siding A").
    #[serde(default)]
    pub track_name: Option<String>,
    /// Milepost at offset 0 of this link (miles). If `None`, it is derived from
    /// the previous link's end milepost plus this link's length. Set this
    /// explicitly to represent real milepost gaps/resets.
    #[serde(default)]
    pub start_milepost: Option<f64>,
    /// `+1` if milepost increases along the link, `-1` if it decreases.
    #[serde(default = "default_mp_dir")]
    pub milepost_direction: f64,
}

/// Default milepost direction (increasing).
pub fn default_mp_dir() -> f64 {
    1.0
}

impl Link {
    /// Forward neighbors (primary then alternate, if real).
    pub fn next_links(&self) -> Vec<LinkIdx> {
        let mut v = vec![self.idx_next];
        if !self.idx_next_alt.is_fake() && self.idx_next_alt != self.idx_next {
            v.push(self.idx_next_alt);
        }
        v.into_iter().filter(|i| !i.is_fake()).collect()
    }

    /// Speed limit for a train type at a given offset (m/s).
    pub fn speed_limit_at(&self, train_type: &str, offset: f64) -> Option<f64> {
        let set = self
            .speed_set
            .as_ref()
            .or_else(|| self.speed_sets.get(train_type))?;
        let mut limit = None;
        for sl in &set.speed_limits {
            if offset >= sl.offset_start && offset <= sl.end_offset {
                limit = Some(match limit {
                    Some(l) => f64::min(l, sl.speed),
                    None => sl.speed,
                });
            }
        }
        limit
    }

    /// Convert a milepost (miles) to an offset from this link's start (m).
    ///
    /// Returns `Some(offset)` only when the link declares a
    /// [`start_milepost`](Self::start_milepost) and the milepost falls within
    /// the link's extent `[0, length]`. Otherwise returns `None`.
    pub fn offset_from_milepost(&self, milepost: f64) -> Option<f64> {
        let start_mp = self.start_milepost?;
        let dir = if self.milepost_direction == 0.0 {
            1.0
        } else {
            self.milepost_direction
        };
        let offset = (milepost - start_mp) / dir * METERS_PER_MILE;
        if offset >= 0.0 && offset <= self.length {
            Some(offset)
        } else {
            None
        }
    }
}

/// A named location (terminal) on the network.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Location {
    /// Location identifier.
    pub location_id: String,
    /// Offset along its link (m).
    pub offset: f64,
    /// Link the location sits on.
    pub link_idx: LinkIdx,
}

/// A track network: a collection of links.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Network {
    /// Links indexed by position (index 0 is a sentinel).
    pub links: Vec<Link>,
    /// Optional named locations (terminals) on the network.
    #[serde(default)]
    pub locations: Vec<Location>,
}

impl Network {
    /// Create a network from a vector of links.
    pub fn new(links: Vec<Link>) -> Self {
        Self {
            links,
            locations: Vec::new(),
        }
    }

    /// Look up a location by its identifier.
    pub fn location(&self, id: &str) -> Option<&Location> {
        self.locations.iter().find(|l| l.location_id == id)
    }

    /// Load a network from a YAML or JSON file.
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path)?;
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_lowercase();
        let network: Network = match ext.as_str() {
            "json" => serde_json::from_str(&text)?,
            "yaml" | "yml" => serde_yaml::from_str(&text)?,
            other => bail!("unsupported network file extension: {other}"),
        };
        Ok(network)
    }

    /// Number of links.
    pub fn len(&self) -> usize {
        self.links.len()
    }

    /// Whether the network is empty.
    pub fn is_empty(&self) -> bool {
        self.links.is_empty()
    }

    /// Get a link by index.
    pub fn link(&self, idx: LinkIdx) -> Option<&Link> {
        self.links.get(idx.idx())
    }

    /// Locate a point on the network from a live position.
    ///
    /// Matches links whose `subdivision` equals `subdivision` and, when
    /// `track_name` is non-empty, whose `track_name` equals `track_name`. An
    /// empty `track_name` matches any track within the subdivision. Among the
    /// matching links, the one whose milepost extent contains `milepost` is
    /// selected.
    ///
    /// # Errors
    /// Returns an error when no link matches ("not found") or when more than
    /// one link matches ("ambiguous"). Subdivision and track name are expected
    /// to disambiguate; a match is never silently chosen.
    pub fn locate(&self, subdivision: &str, track_name: &str, milepost: f64) -> Result<Location> {
        let mut matches: Vec<Location> = Vec::new();
        for link in &self.links {
            if link.idx_curr.is_fake() {
                continue;
            }
            if link.subdivision.as_deref() != Some(subdivision) {
                continue;
            }
            if !track_name.is_empty() && link.track_name.as_deref() != Some(track_name) {
                continue;
            }
            if let Some(offset) = link.offset_from_milepost(milepost) {
                let track = if track_name.is_empty() {
                    link.track_name.as_deref().unwrap_or("")
                } else {
                    track_name
                };
                matches.push(Location {
                    location_id: format!("{subdivision}/{track}@{milepost}"),
                    offset,
                    link_idx: link.idx_curr,
                });
            }
        }
        match matches.len() {
            0 => bail!(
                "no link found for subdivision '{subdivision}', track '{track_name}', milepost {milepost}"
            ),
            1 => Ok(matches.pop().unwrap()),
            n => bail!(
                "ambiguous live position: {n} links match subdivision '{subdivision}', track '{track_name}', milepost {milepost}"
            ),
        }
    }
}
