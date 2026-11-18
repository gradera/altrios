//! Tests for `Network::locate` (live-position lookup).

use route_engine::{Link, LinkIdx, Network};

const M_PER_MI: f64 = 1609.344;

/// Build a link with an explicit subdivision, track name, and start milepost.
fn link(idx: u32, next: u32, sub: &str, track: &str, start_mp: f64) -> Link {
    Link {
        idx_curr: LinkIdx(idx),
        idx_next: LinkIdx(next),
        length: M_PER_MI,
        subdivision: Some(sub.to_string()),
        track_name: Some(track.to_string()),
        start_milepost: Some(start_mp),
        ..Default::default()
    }
}

/// Network with two subdivisions and a siding alongside the main.
fn net() -> Network {
    Network::new(vec![
        Link::default(),
        // Sub A, Main: MP 0..1
        link(1, 2, "Sub A", "Main", 0.0),
        // Sub A, Main: MP 1..2
        link(2, 3, "Sub A", "Main", 1.0),
        // Sub B, Main: MP 2..3
        link(3, 0, "Sub B", "Main", 2.0),
        // Sub A, Siding: MP 0..1 (same MP range as link 1)
        link(4, 0, "Sub A", "Siding", 0.0),
    ])
}

#[test]
fn locate_found_on_main() {
    let net = net();
    let loc = net.locate("Sub A", "Main", 1.5).unwrap();
    assert_eq!(loc.link_idx, LinkIdx(2));
    // Half a mile into link 2 (which starts at MP 1).
    assert!((loc.offset - 0.5 * M_PER_MI).abs() < 1e-6);
}

#[test]
fn locate_empty_track_matches_any_track() {
    // Sub A @ MP 0.5 is ambiguous across Main and Siding when track is empty.
    let net = net();
    let err = net.locate("Sub A", "", 0.5).unwrap_err().to_string();
    assert!(err.contains("ambiguous"), "unexpected error: {err}");

    // Sub B has only Main, so an empty track resolves unambiguously.
    let loc = net.locate("Sub B", "", 2.5).unwrap();
    assert_eq!(loc.link_idx, LinkIdx(3));
}

#[test]
fn locate_not_found() {
    let net = net();
    // MP 5 is outside every link's extent.
    let err = net.locate("Sub A", "Main", 5.0).unwrap_err().to_string();
    assert!(err.contains("no link found"), "unexpected error: {err}");

    // Unknown subdivision.
    let err = net.locate("Sub Z", "Main", 0.5).unwrap_err().to_string();
    assert!(err.contains("no link found"), "unexpected error: {err}");
}

#[test]
fn locate_ambiguous_within_same_track() {
    // Two overlapping Sub A Main segments (a modeling mistake) -> ambiguous.
    let mut links = vec![
        Link::default(),
        link(1, 0, "Sub A", "Main", 0.0),
        link(2, 0, "Sub A", "Main", 0.0),
    ];
    links[2].idx_next = LinkIdx(0);
    let net = Network::new(links);
    let err = net.locate("Sub A", "Main", 0.5).unwrap_err().to_string();
    assert!(err.contains("ambiguous"), "unexpected error: {err}");
}

#[test]
fn offset_from_milepost_respects_direction() {
    // Decreasing milepost direction: MP 2 at offset 0, MP 1 at offset 1 mi.
    let mut l = link(1, 0, "Sub A", "Main", 2.0);
    l.milepost_direction = -1.0;
    assert!((l.offset_from_milepost(2.0).unwrap() - 0.0).abs() < 1e-6);
    assert!((l.offset_from_milepost(1.5).unwrap() - 0.5 * M_PER_MI).abs() < 1e-6);
    assert!(l.offset_from_milepost(3.0).is_none());
    assert!(l.offset_from_milepost(0.5).is_none());
}

#[test]
fn offset_from_milepost_requires_start() {
    let mut l = link(1, 0, "Sub A", "Main", 0.0);
    l.start_milepost = None;
    assert!(l.offset_from_milepost(0.5).is_none());
}
