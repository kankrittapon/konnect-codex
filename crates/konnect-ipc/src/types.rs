use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcVector2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcFootprint {
    pub reference: String,
    pub value: String,
    pub footprint: String,
    pub position: IpcVector2,
    pub rotation: f64,
    pub layer: String,
}

#[derive(Debug, Clone)]
pub struct IpcPadDefinition {
    pub number: String,
    pub pad_type: String,
    pub shape: String,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
    pub size_x: f64,
    pub size_y: f64,
    pub drill_x: Option<f64>,
    pub drill_y: Option<f64>,
    pub drill_oval: bool,
    pub layers: Vec<String>,
    pub roundrect_ratio: f64,
}

/// A footprint graphic item in footprint-local coordinates (mm), parsed from
/// the library `.kicad_mod` source.
///
/// Points are pre-transform: `build_footprint_item` rotates and translates
/// them into absolute board coordinates before emission, because KiCAD
/// serializes `FootprintInstance` children in absolute board space (see the
/// `transform` module docs / issue #23).
#[derive(Debug, Clone, PartialEq)]
pub enum IpcGraphicDefinition {
    /// `fp_line` — straight segment.
    Line {
        start: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
    },
    /// `fp_rect` — axis-aligned rectangle between two opposite corners.
    Rect {
        start: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
        filled: bool,
    },
    /// `fp_circle` — center plus a point on the circumference.
    Circle {
        center: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
        filled: bool,
    },
    /// `fp_arc` — start / mid / end points.
    Arc {
        start: (f64, f64),
        mid: (f64, f64),
        end: (f64, f64),
        layer: String,
        width: f64,
    },
    /// `fp_poly` — closed outline.
    Poly {
        points: Vec<(f64, f64)>,
        layer: String,
        width: f64,
        filled: bool,
    },
    /// Visible `fp_text` / `property` text.
    Text {
        text: String,
        position: (f64, f64),
        /// Text angle in degrees, footprint-local.
        rotation: f64,
        layer: String,
        /// Glyph size (width and height) in mm.
        size: f64,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcTrack {
    /// KiCad-native stable object identity (KIID).
    pub kiid: String,
    pub net_name: String,
    pub layer: String,
    pub width: f64,
    pub start: IpcVector2,
    pub end: IpcVector2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcTrackArc {
    pub net_name: String,
    pub layer: String,
    pub width: f64,
    pub start: IpcVector2,
    pub mid: IpcVector2,
    pub end: IpcVector2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcNet {
    pub name: String,
    pub netcode: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcLayer {
    pub name: String,
    pub id: i32,
    pub kind: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcBoardExtents {
    pub min: IpcVector2,
    pub max: IpcVector2,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcPadGeometry {
    pub reference: String,
    pub number: String,
    pub position: IpcVector2,
    pub size: Option<IpcVector2>,
    pub shape: Option<String>,
    pub rotation: f64,
    pub corner_rounding_ratio: Option<f64>,
    pub geometry_supported: bool,
    pub geometry_reason: Option<String>,
    pub layers: Vec<String>,
    pub net_name: String,
    pub net_id: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum IpcBoardEdgeGeometry {
    Line {
        start: IpcVector2,
        end: IpcVector2,
    },
    Arc {
        start: IpcVector2,
        mid: IpcVector2,
        end: IpcVector2,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcViaGeometry {
    /// KiCad-native stable object identity (KIID).
    pub kiid: String,
    pub position: IpcVector2,
    pub size: Option<IpcVector2>,
    pub drill: Option<IpcVector2>,
    pub layers: Vec<String>,
    pub net_name: String,
    pub net_id: i32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcPolygonGeometry {
    pub object_type: String,
    pub reference: Option<String>,
    pub layer: String,
    pub net_name: Option<String>,
    pub net_id: Option<i32>,
    pub polygons: Vec<Vec<IpcVector2>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcGeometryClass<T> {
    pub available: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    pub items: T,
}

impl<T: Default> IpcGeometryClass<T> {
    pub fn available(items: T) -> Self {
        Self {
            available: true,
            reason: None,
            items,
        }
    }

    pub fn unavailable(reason: impl Into<String>) -> Self {
        Self {
            available: false,
            reason: Some(reason.into()),
            items: T::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IpcRoutingGeometry {
    pub layers: IpcGeometryClass<Vec<IpcLayer>>,
    pub board_extents: IpcGeometryClass<Option<IpcBoardExtents>>,
    pub footprints: IpcGeometryClass<Vec<IpcFootprint>>,
    pub pads: IpcGeometryClass<Vec<IpcPadGeometry>>,
    pub tracks: IpcGeometryClass<Vec<IpcTrack>>,
    pub track_arcs: IpcGeometryClass<Vec<IpcTrackArc>>,
    pub vias: IpcGeometryClass<Vec<IpcViaGeometry>>,
    pub courtyards: IpcGeometryClass<Vec<IpcPolygonGeometry>>,
    pub zones: IpcGeometryClass<Vec<IpcPolygonGeometry>>,
    pub board_edges: IpcGeometryClass<Vec<IpcBoardEdgeGeometry>>,
}

/// Read-only identity for a board item referenced by a KiCad DRC report.
/// Optional fields stay absent when the corresponding protobuf item does not
/// carry a single, exact value (for example, a multi-layer zone).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct IpcBoardItemIdentity {
    pub kiid: String,
    pub item_type: String,
    pub reference: Option<String>,
    pub pad: Option<String>,
    pub net: Option<String>,
    pub layer: Option<String>,
    pub position: Option<IpcVector2>,
}

/// Footprint-local placement of the Reference and Value text fields, read
/// from the library footprint so placed parts keep the library's text
/// positions. A hardcoded offset put the Reference on top of the part's own
/// silkscreen (silk_overlap DRC warnings in live verification).
#[derive(Debug, Clone, Copy, Default)]
pub struct IpcFieldPlacement {
    /// (x, y, rotation) of the Reference text, footprint-local mm/degrees.
    pub reference_at: Option<(f64, f64, f64)>,
    /// (x, y, rotation) of the Value text, footprint-local mm/degrees.
    pub value_at: Option<(f64, f64, f64)>,
}
