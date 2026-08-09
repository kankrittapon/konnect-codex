//! `pcb_routing` toolset — traces, vias, copper pours, nets, netclasses, and diff pairs.
//!
//! Routing operations use the KiCAD IPC API; `add_net`, `create_netclass`, and
//! `add_copper_pour` use S-expression file manipulation.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::{get_path, require_f64, require_str, ToolContext, ToolDef};
use konnect_ipc::client::KiCadIpcClient;
use konnect_ipc::{IpcBoardEdgeGeometry, IpcRoutingGeometry, IpcVector2};
use konnect_sexp::writer::{apply_edits, new_uuid, write_atomic, SexpEdit};
use serde_json::json;

#[derive(Debug, Clone, PartialEq, Eq)]
struct AllowedEndpointPad {
    reference: String,
    pad: String,
}

struct ClearanceReport {
    conflicts: Vec<serde_json::Value>,
    courtyard_intersections: Vec<serde_json::Value>,
    minimum: f64,
}

// ─── IPC helper ───────────────────────────────────────────────────────────────

async fn with_ipc<T, F>(addr: String, f: F) -> anyhow::Result<Result<T, String>>
where
    T: Send + 'static,
    F: FnOnce(&KiCadIpcClient) -> anyhow::Result<T> + Send + 'static,
{
    match tokio::task::spawn_blocking(move || f(&KiCadIpcClient::new(&addr))).await {
        Ok(Ok(r)) => Ok(Ok(r)),
        Ok(Err(e)) => Ok(Err(e.to_string())),
        Err(e) => Err(anyhow::anyhow!("Thread error: {}", e)),
    }
}

macro_rules! ipc {
    ($ctx:expr, |$c:ident| $body:expr) => {{
        let addr = $ctx.config.ipc_address.clone();
        match with_ipc(addr, move |$c| $body).await? {
            Ok(v) => v,
            Err(msg) => {
                return Ok(CallToolResult::error(format!(
                    "KiCAD must be running with the board loaded (IPC error: {})",
                    msg
                )))
            }
        }
    }};
}

// ─── S-expression helpers ─────────────────────────────────────────────────────

fn format_zone(
    net_id: i32,
    net_name: &str,
    layer: &str,
    clearance: f64,
    min_w: f64,
    pts: &[(f64, f64)],
) -> String {
    let uuid = new_uuid();
    let pt_str: String = pts
        .iter()
        .map(|(x, y)| format!("\n      (xy {x} {y})"))
        .collect();
    format!(
        "\n  (zone (net {net_id}) (net_name \"{net_name}\") (layer \"{layer}\") (uuid \"{uuid}\")\n    \
         (hatch edge 0.508)\n    (connect_pads (clearance {clearance}))\n    \
         (min_thickness {min_w})\n    (fill yes)\n    \
         (polygon (pts{pt_str}\n    ))\n  )"
    )
}

fn find_net_id(content: &str, net_name: &str) -> i32 {
    let search = format!(r#" "{net_name}")"#);
    if let Some(pos) = content.find(&search) {
        let before = &content[..pos];
        let net_pos = before.rfind("(net ").unwrap_or(0);
        let num_str = &before[net_pos + 5..];
        let num_end = num_str.find(' ').unwrap_or(0);
        num_str[..num_end].parse().unwrap_or(0)
    } else {
        0
    }
}

// ─── Tool definitions ─────────────────────────────────────────────────────────

pub fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "get_routing_geometry",
            "Read PCB routing geometry from the active KiCAD board without modifying it.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "layer": { "type": "string", "description": "Optional layer filter" },
                    "net": { "type": "string", "description": "Optional net-name filter" },
                    "region": { "type": "object", "properties": { "x_min": {"type":"number"}, "y_min": {"type":"number"}, "x_max": {"type":"number"}, "y_max": {"type":"number"} }, "required": ["x_min","y_min","x_max","y_max"] }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_get_routing_geometry(args, ctx).await }
        ),
        tool!(
            "check_route_clearance",
            "Read-only geometric preflight for a proposed PCB trace polyline; never creates or changes board items.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "points": { "type": "array", "minItems": 2, "items": { "type": "object", "properties": { "x": {"type":"number"}, "y": {"type":"number"} }, "required": ["x","y"] } },
                    "layer": { "type": "string" },
                    "trace_width": { "type": "number", "exclusiveMinimum": 0 },
                    "net": { "type": "string" },
                    "required_clearance": { "type": "number", "minimum": 0 },
                    "allowed_endpoint_pads": {
                        "type": "array",
                        "description": "Exact pads that may contact a corresponding route start or end point; does not exempt neighboring pads or middle-of-route crossings.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "reference": { "type": "string" },
                                "pad": { "type": "string" }
                            },
                            "required": ["reference", "pad"],
                            "additionalProperties": false
                        }
                    }
                },
                "required": ["board","points","layer","trace_width","net","required_clearance"]
            }),
            |args, ctx| async move { handle_check_route_clearance(args, ctx).await }
        ),
        tool!(
            "check_via_clearance",
            "Read-only preflight for a proposed via across every traversed copper layer; never creates a via.",
            json!({"type":"object","properties":{
                "board":{"type":"string"},"x":{"type":"number"},"y":{"type":"number"},
                "net":{"type":"string"},"diameter":{"type":"number","exclusiveMinimum":0},
                "drill":{"type":"number","exclusiveMinimum":0},"start_layer":{"type":"string"},
                "end_layer":{"type":"string"},"required_clearance":{"type":"number","minimum":0}
            },"required":["board","x","y","net","diameter","drill","start_layer","end_layer","required_clearance"]}),
            |args, ctx| async move { handle_check_via_clearance(args, ctx).await }
        ),
        tool!(
            "add_net",
            "Add a new net entry to the PCB file (S-expression insert, no KiCAD IPC required).",
            json!({
                "type": "object",
                "properties": {
                    "board":    { "type": "string" },
                    "net_name": { "type": "string" }
                },
                "required": ["board", "net_name"]
            }),
            |args, ctx| async move { handle_add_net(args, ctx).await }
        ),
        tool!(
            "route_trace",
            "Route a trace segment between two points on a copper layer via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":    { "type": "string" },
                    "net_name": { "type": "string" },
                    "layer":    { "type": "string", "description": "Copper layer (e.g. 'F.Cu')" },
                    "x1": { "type": "number" }, "y1": { "type": "number" },
                    "x2": { "type": "number" }, "y2": { "type": "number" },
                    "width": { "type": "number", "default": 0.25 }
                },
                "required": ["board", "net_name", "layer", "x1", "y1", "x2", "y2"]
            }),
            |args, ctx| async move { handle_route_trace(args, ctx).await }
        ),
        tool!(
            "route_pad_to_pad",
            "Route a direct trace between two pads of named components (L-bend routing) via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":       { "type": "string" },
                    "net_name":    { "type": "string" },
                    "ref1":        { "type": "string", "description": "First component reference" },
                    "pad1":        { "type": "string", "description": "First pad number" },
                    "ref2":        { "type": "string", "description": "Second component reference" },
                    "pad2":        { "type": "string", "description": "Second pad number" },
                    "layer":       { "type": "string", "default": "F.Cu" },
                    "width":       { "type": "number", "default": 0.25 }
                },
                "required": ["board", "net_name", "ref1", "pad1", "ref2", "pad2"]
            }),
            |args, ctx| async move { handle_route_pad_to_pad(args, ctx).await }
        ),
        tool!(
            "add_via",
            "Add a through-hole via at a given position and assign it to a net via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "net_name":  { "type": "string" },
                    "x":         { "type": "number" },
                    "y":         { "type": "number" },
                    "drill":     { "type": "number", "description": "Drill diameter in mm", "default": 0.4 },
                    "pad_size":  { "type": "number", "description": "Via pad diameter in mm", "default": 0.8 }
                },
                "required": ["board", "net_name", "x", "y"]
            }),
            |args, ctx| async move { handle_add_via(args, ctx).await }
        ),
        tool!(
            "add_copper_pour",
            "Add a copper fill zone polygon on a layer/net via S-expression file insert.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "net_name":  { "type": "string" },
                    "layer":     { "type": "string", "description": "Copper layer (e.g. 'F.Cu')" },
                    "points": {
                        "type": "array",
                        "items": { "type": "object", "properties": { "x": { "type": "number" }, "y": { "type": "number" } } }
                    },
                    "clearance": { "type": "number", "default": 0.2 },
                    "min_width": { "type": "number", "default": 0.25 }
                },
                "required": ["board", "net_name", "layer", "points"]
            }),
            |args, ctx| async move { handle_add_copper_pour(args, ctx).await }
        ),
        tool!(
            "delete_trace",
            "Delete a trace segment identified by its UUID via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" },
                    "uuid":  { "type": "string", "description": "UUID of the track segment to delete" }
                },
                "required": ["board", "uuid"]
            }),
            |args, ctx| async move { handle_delete_trace(args, ctx).await }
        ),
        tool!(
            "query_traces",
            "List trace segments on the board, optionally filtered by net and/or layer.",
            json!({
                "type": "object",
                "properties": {
                    "board":    { "type": "string" },
                    "net_name": { "type": "string", "description": "Filter by net (optional)" },
                    "layer":    { "type": "string", "description": "Filter by layer (optional)" }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_query_traces(args, ctx).await }
        ),
        tool!(
            "get_nets_list",
            "Return all nets defined on the PCB via KiCAD IPC.",
            json!({
                "type": "object",
                "properties": {
                    "board": { "type": "string" }
                },
                "required": ["board"]
            }),
            |args, ctx| async move { handle_get_nets_list(args, ctx).await }
        ),
        tool!(
            "modify_trace",
            "Modify a trace segment by deleting and re-adding it with new parameters.",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string" },
                    "uuid":      { "type": "string" },
                    "net_name":  { "type": "string" },
                    "layer":     { "type": "string" },
                    "x1": { "type": "number" }, "y1": { "type": "number" },
                    "x2": { "type": "number" }, "y2": { "type": "number" },
                    "width":     { "type": "number", "default": 0.25 }
                },
                "required": ["board", "uuid", "net_name", "layer", "x1", "y1", "x2", "y2"]
            }),
            |args, ctx| async move { handle_modify_trace(args, ctx).await }
        ),
        tool!(
            "create_netclass",
            "Add a netclass definition to the board's design rules (S-expression file insert).",
            json!({
                "type": "object",
                "properties": {
                    "board":        { "type": "string" },
                    "name":         { "type": "string", "description": "Netclass name (e.g. 'Power')" },
                    "clearance":    { "type": "number", "description": "Clearance in mm", "default": 0.2 },
                    "trace_width":  { "type": "number", "description": "Default trace width in mm", "default": 0.25 },
                    "via_drill":    { "type": "number", "description": "Via drill diameter in mm", "default": 0.4 },
                    "via_diameter": { "type": "number", "description": "Via pad diameter in mm", "default": 0.8 }
                },
                "required": ["board", "name"]
            }),
            |args, ctx| async move { handle_create_netclass(args, ctx).await }
        ),
        tool!(
            "assign_net_to_class",
            "Assign a net to an existing netclass in the PCB file (S-expression edit).",
            json!({
                "type": "object",
                "properties": {
                    "board":     { "type": "string", "description": "Path to .kicad_pcb file" },
                    "net_name":  { "type": "string", "description": "Net name to assign" },
                    "netclass":  { "type": "string", "description": "Netclass name to assign the net to" }
                },
                "required": ["board", "net_name", "netclass"]
            }),
            |args, ctx| async move { handle_assign_net_to_class(args, ctx).await }
        ),
        tool!(
            "route_differential_pair",
            "Route a differential pair (two parallel traces with a specified gap).",
            json!({
                "type": "object",
                "properties": {
                    "board":    { "type": "string" },
                    "net_pos":  { "type": "string", "description": "Positive net name" },
                    "net_neg":  { "type": "string", "description": "Negative net name" },
                    "layer":    { "type": "string", "default": "F.Cu" },
                    "x1": { "type": "number" }, "y1": { "type": "number" },
                    "x2": { "type": "number" }, "y2": { "type": "number" },
                    "width": { "type": "number", "default": 0.1 },
                    "gap":   { "type": "number", "description": "Gap between pair traces in mm", "default": 0.1 }
                },
                "required": ["board", "net_pos", "net_neg", "x1", "y1", "x2", "y2"]
            }),
            |args, ctx| async move { handle_route_diff_pair(args, ctx).await }
        ),
    ]
}

// ─── Handlers ─────────────────────────────────────────────────────────────────

fn parse_points(args: &serde_json::Value) -> Result<Vec<(f64, f64)>, CallToolResult> {
    let Some(values) = args["points"].as_array() else {
        return Err(CallToolResult::error("Missing 'points' array"));
    };
    let points: Option<Vec<_>> = values
        .iter()
        .map(|p| Some((p["x"].as_f64()?, p["y"].as_f64()?)))
        .collect();
    let Some(points) = points else {
        return Err(CallToolResult::error(
            "Every point requires numeric x and y",
        ));
    };
    if points.len() < 2 {
        return Err(CallToolResult::error(
            "At least two ordered points are required",
        ));
    }
    Ok(points)
}

fn parse_allowed_endpoint_pads(
    args: &serde_json::Value,
) -> Result<Vec<AllowedEndpointPad>, CallToolResult> {
    let Some(value) = args.get("allowed_endpoint_pads") else {
        return Ok(Vec::new());
    };
    let Some(values) = value.as_array() else {
        return Err(CallToolResult::error(
            "allowed_endpoint_pads must be an array",
        ));
    };
    values
        .iter()
        .map(|value| {
            let reference = value["reference"].as_str().filter(|v| !v.is_empty());
            let pad = value["pad"].as_str().filter(|v| !v.is_empty());
            match (reference, pad) {
                (Some(reference), Some(pad)) => Ok(AllowedEndpointPad {
                    reference: reference.to_string(),
                    pad: pad.to_string(),
                }),
                _ => Err(CallToolResult::error(
                    "Every allowed endpoint pad requires non-empty reference and pad strings",
                )),
            }
        })
        .collect()
}

fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let ab = (b.0 - a.0, b.1 - a.1);
    let denom = ab.0 * ab.0 + ab.1 * ab.1;
    if denom == 0.0 {
        return ((p.0 - a.0).powi(2) + (p.1 - a.1).powi(2)).sqrt();
    }
    let t = (((p.0 - a.0) * ab.0 + (p.1 - a.1) * ab.1) / denom).clamp(0.0, 1.0);
    ((p.0 - (a.0 + t * ab.0)).powi(2) + (p.1 - (a.1 + t * ab.1)).powi(2)).sqrt()
}

fn segment_distance(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> f64 {
    fn orient(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    }
    fn on_segment(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> bool {
        const EPSILON: f64 = 1e-12;
        p.0 >= a.0.min(b.0) - EPSILON
            && p.0 <= a.0.max(b.0) + EPSILON
            && p.1 >= a.1.min(b.1) - EPSILON
            && p.1 <= a.1.max(b.1) + EPSILON
    }
    let o = [
        orient(a, b, c),
        orient(a, b, d),
        orient(c, d, a),
        orient(c, d, b),
    ];
    let proper_crossing = o[0] * o[1] < 0.0 && o[2] * o[3] < 0.0;
    let endpoint_crossing = (o[0].abs() <= 1e-12 && on_segment(a, b, c))
        || (o[1].abs() <= 1e-12 && on_segment(a, b, d))
        || (o[2].abs() <= 1e-12 && on_segment(c, d, a))
        || (o[3].abs() <= 1e-12 && on_segment(c, d, b));
    if proper_crossing || endpoint_crossing {
        0.0
    } else {
        point_segment_distance(a, c, d)
            .min(point_segment_distance(b, c, d))
            .min(point_segment_distance(c, a, b))
            .min(point_segment_distance(d, a, b))
    }
}

fn pad_bounding_box(pad: &konnect_ipc::IpcPadGeometry) -> Option<(f64, f64, f64, f64)> {
    let size = pad.size.as_ref()?;
    let radians = pad.rotation.to_radians();
    let (sin, cos) = radians.sin_cos();
    let half_x = size.x / 2.0;
    let half_y = size.y / 2.0;
    let extent_x = half_x * cos.abs() + half_y * sin.abs();
    let extent_y = half_x * sin.abs() + half_y * cos.abs();
    Some((
        pad.position.x - extent_x,
        pad.position.y - extent_y,
        pad.position.x + extent_x,
        pad.position.y + extent_y,
    ))
}

fn pad_intersects_segment_search_envelope(
    pad: &konnect_ipc::IpcPadGeometry,
    a: (f64, f64),
    b: (f64, f64),
    expansion: f64,
) -> bool {
    let Some((pad_x_min, pad_y_min, pad_x_max, pad_y_max)) = pad_bounding_box(pad) else {
        return true;
    };
    let trace_x_min = a.0.min(b.0) - expansion;
    let trace_y_min = a.1.min(b.1) - expansion;
    let trace_x_max = a.0.max(b.0) + expansion;
    let trace_y_max = a.1.max(b.1) + expansion;
    pad_x_max >= trace_x_min
        && pad_x_min <= trace_x_max
        && pad_y_max >= trace_y_min
        && pad_y_min <= trace_y_max
}

fn rotate_into_local(p: (f64, f64), center: (f64, f64), degrees: f64) -> (f64, f64) {
    let r = (-degrees).to_radians();
    let (s, c) = r.sin_cos();
    let (x, y) = (p.0 - center.0, p.1 - center.1);
    (x * c - y * s, x * s + y * c)
}

fn segment_rect_distance(a: (f64, f64), b: (f64, f64), hx: f64, hy: f64) -> f64 {
    if (a.0.abs() <= hx && a.1.abs() <= hy) || (b.0.abs() <= hx && b.1.abs() <= hy) {
        return 0.0;
    }
    let corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)];
    corners
        .windows(2)
        .map(|e| segment_distance(a, b, e[0], e[1]))
        .fold(f64::INFINITY, f64::min)
}

fn pad_segment_distance(
    p: &konnect_ipc::IpcPadGeometry,
    a: (f64, f64),
    b: (f64, f64),
) -> Option<f64> {
    if !p.geometry_supported {
        return None;
    }
    let size = p.size.as_ref()?;
    let a = rotate_into_local(a, (p.position.x, p.position.y), p.rotation);
    let b = rotate_into_local(b, (p.position.x, p.position.y), p.rotation);
    let hx = size.x / 2.0;
    let hy = size.y / 2.0;
    match p.shape.as_deref()? {
        "PssCircle" | "circle" => Some(point_segment_distance((0.0, 0.0), a, b) - hx),
        "PssRectangle" | "rectangle" => Some(segment_rect_distance(a, b, hx, hy)),
        "PssOval" | "oval" => {
            let (c, d, radius) = if hx >= hy {
                ((-hx + hy, 0.0), (hx - hy, 0.0), hy)
            } else {
                ((0.0, -hy + hx), (0.0, hy - hx), hx)
            };
            Some(segment_distance(a, b, c, d) - radius)
        }
        "PssRoundrect" | "roundrect" => {
            let radius = p.corner_rounding_ratio.unwrap_or(0.0) * size.x.min(size.y);
            Some(
                segment_rect_distance(a, b, (hx - radius).max(0.0), (hy - radius).max(0.0))
                    - radius,
            )
        }
        _ => None,
    }
    .map(|d| d.max(0.0))
}

fn circle_from_three(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<((f64, f64), f64)> {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    if d.abs() < 1e-12 {
        return None;
    }
    let aa = a.0 * a.0 + a.1 * a.1;
    let bb = b.0 * b.0 + b.1 * b.1;
    let cc = c.0 * c.0 + c.1 * c.1;
    let center = (
        (aa * (b.1 - c.1) + bb * (c.1 - a.1) + cc * (a.1 - b.1)) / d,
        (aa * (c.0 - b.0) + bb * (a.0 - c.0) + cc * (b.0 - a.0)) / d,
    );
    Some((
        center,
        ((a.0 - center.0).powi(2) + (a.1 - center.1).powi(2)).sqrt(),
    ))
}

fn angle_on_arc(angle: f64, start: f64, mid: f64, end: f64) -> bool {
    let norm = |x: f64| x.rem_euclid(std::f64::consts::TAU);
    let ccw = |a: f64, b: f64| norm(b - a);
    let (s, m, e, x) = (norm(start), norm(mid), norm(end), norm(angle));
    if ccw(s, m) <= ccw(s, e) + 1e-10 {
        ccw(s, x) <= ccw(s, e) + 1e-10
    } else {
        ccw(e, x) <= ccw(e, s) + 1e-10
    }
}

fn segment_arc_distance(
    a: (f64, f64),
    b: (f64, f64),
    start: (f64, f64),
    mid: (f64, f64),
    end: (f64, f64),
) -> f64 {
    let Some((center, r)) = circle_from_three(start, mid, end) else {
        return segment_distance(a, b, start, end);
    };
    let angles = (
        (start.1 - center.1).atan2(start.0 - center.0),
        (mid.1 - center.1).atan2(mid.0 - center.0),
        (end.1 - center.1).atan2(end.0 - center.0),
    );
    let mut best = point_segment_distance(start, a, b).min(point_segment_distance(end, a, b));
    let ab = (b.0 - a.0, b.1 - a.1);
    let den = ab.0 * ab.0 + ab.1 * ab.1;
    if den > 0.0 {
        let t = (((center.0 - a.0) * ab.0 + (center.1 - a.1) * ab.1) / den).clamp(0.0, 1.0);
        for p in [a, b, (a.0 + t * ab.0, a.1 + t * ab.1)] {
            let ang = (p.1 - center.1).atan2(p.0 - center.0);
            if angle_on_arc(ang, angles.0, angles.1, angles.2) {
                best = best
                    .min((((p.0 - center.0).powi(2) + (p.1 - center.1).powi(2)).sqrt() - r).abs());
            }
        }
    }
    best
}

fn geometry_in_region(p: &IpcVector2, region: Option<(f64, f64, f64, f64)>) -> bool {
    region
        .map(|r| p.x >= r.0 && p.y >= r.1 && p.x <= r.2 && p.y <= r.3)
        .unwrap_or(true)
}

fn parse_region(args: &serde_json::Value) -> Result<Option<(f64, f64, f64, f64)>, CallToolResult> {
    let Some(value) = args.get("region") else {
        return Ok(None);
    };
    let Some(region) = value.as_object() else {
        return Err(CallToolResult::error("region must be an object"));
    };
    let coordinate = |name: &str| {
        region
            .get(name)
            .and_then(serde_json::Value::as_f64)
            .filter(|value| value.is_finite())
            .ok_or_else(|| CallToolResult::error(format!("region.{name} must be finite")))
    };
    let parsed = (
        coordinate("x_min")?,
        coordinate("y_min")?,
        coordinate("x_max")?,
        coordinate("y_max")?,
    );
    if parsed.0 >= parsed.2 || parsed.1 >= parsed.3 {
        return Err(CallToolResult::error(
            "region requires x_min < x_max and y_min < y_max",
        ));
    }
    Ok(Some(parsed))
}

fn board_edge_check_status(g: &IpcRoutingGeometry) -> serde_json::Value {
    if g.board_edges.available {
        json!({"supported":true,"method":"exact KiCad IPC Edge.Cuts line/arc primitives","primitive_count":g.board_edges.items.len()})
    } else {
        json!({"supported":false,"reason":g.board_edges.reason.as_deref().unwrap_or("Edge.Cuts geometry unavailable")})
    }
}

fn endpoint_is_inside_pad(endpoint: (f64, f64), pad: &konnect_ipc::IpcPadGeometry) -> bool {
    pad_segment_distance(pad, endpoint, endpoint) == Some(0.0)
}

fn pad_is_allowed_at_segment_endpoint(
    pad: &konnect_ipc::IpcPadGeometry,
    allowed: &[AllowedEndpointPad],
    points: &[(f64, f64)],
    segment_index: usize,
) -> bool {
    if !allowed
        .iter()
        .any(|item| item.reference == pad.reference && item.pad == pad.number)
    {
        return false;
    }
    (segment_index == 0 && endpoint_is_inside_pad(points[0], pad))
        || (segment_index + 2 == points.len()
            && endpoint_is_inside_pad(*points.last().expect("route has two points"), pad))
}

fn calculate_clearance(
    g: &IpcRoutingGeometry,
    points: &[(f64, f64)],
    layer: &str,
    net: &str,
    width: f64,
    clearance: f64,
    allowed_endpoint_pads: &[AllowedEndpointPad],
) -> ClearanceReport {
    let mut conflicts = Vec::new();
    let mut courtyard_intersections = Vec::new();
    let mut minimum = f64::INFINITY;
    for (segment_index, pair) in points.windows(2).enumerate() {
        let (a, b) = (pair[0], pair[1]);
        for t in g
            .tracks
            .items
            .iter()
            .filter(|t| t.layer == layer && t.net_name != net)
        {
            let center = segment_distance(a, b, (t.start.x, t.start.y), (t.end.x, t.end.y));
            let gap = center - (width + t.width) / 2.0;
            minimum = minimum.min(gap);
            if gap < clearance {
                conflicts.push(json!({"object_type":"track","net":t.net_name,"layer":t.layer,"approximate_clearance":gap,"segment":{"start":t.start,"end":t.end},"required_clearance":clearance}));
            }
        }
        for p in g.pads.items.iter().filter(|p| {
            p.net_name != net && (p.layers.is_empty() || p.layers.iter().any(|l| l == layer))
        }) {
            if !pad_intersects_segment_search_envelope(p, a, b, width / 2.0 + clearance) {
                continue;
            }
            if pad_is_allowed_at_segment_endpoint(p, allowed_endpoint_pads, points, segment_index) {
                continue;
            }
            let Some(distance) = pad_segment_distance(p, a, b) else {
                continue;
            };
            let gap = distance - width / 2.0;
            minimum = minimum.min(gap);
            if gap < clearance {
                conflicts.push(json!({"object_type":"pad","reference":p.reference,"pad":p.number,"net":p.net_name,"position":p.position,"actual_clearance":gap,"required_clearance":clearance}));
            }
        }
        for v in g.vias.items.iter().filter(|v| v.net_name != net) {
            let radius = v.size.as_ref().map(|s| s.x.max(s.y) / 2.0).unwrap_or(0.0);
            let gap =
                point_segment_distance((v.position.x, v.position.y), a, b) - width / 2.0 - radius;
            minimum = minimum.min(gap);
            if gap < clearance {
                conflicts.push(json!({"object_type":"via","net":v.net_name,"position":v.position,"approximate_clearance":gap,"required_clearance":clearance}));
            }
        }
        for edge in &g.board_edges.items {
            let distance = match edge {
                IpcBoardEdgeGeometry::Line { start, end } => {
                    segment_distance(a, b, (start.x, start.y), (end.x, end.y))
                }
                IpcBoardEdgeGeometry::Arc { start, mid, end } => {
                    segment_arc_distance(a, b, (start.x, start.y), (mid.x, mid.y), (end.x, end.y))
                }
            };
            let gap = distance - width / 2.0;
            minimum = minimum.min(gap);
            if gap < clearance {
                conflicts.push(json!({"object_type":"board_edge","geometry":edge,"actual_clearance":gap,"required_clearance":clearance}));
            }
        }
        for c in g.courtyards.items.iter().filter(|c| {
            c.layer
                == if layer == "B.Cu" {
                    "B.CrtYd"
                } else {
                    "F.CrtYd"
                }
        }) {
            for poly in &c.polygons {
                for edge in poly.windows(2) {
                    let gap =
                        segment_distance(a, b, (edge[0].x, edge[0].y), (edge[1].x, edge[1].y))
                            - width / 2.0;
                    if gap < clearance {
                        courtyard_intersections.push(json!({"reference":c.reference,"layer":c.layer,"approximate_clearance":gap,"requested_copper_clearance":clearance,"segment_index":segment_index}));
                    }
                }
                if poly.len() >= 3
                    && (point_in_polygon((a.0, a.1), poly) || point_in_polygon((b.0, b.1), poly))
                {
                    courtyard_intersections.push(json!({"reference":c.reference,"layer":c.layer,"approximate_clearance":0.0,"requested_copper_clearance":clearance,"segment_index":segment_index,"informational":"route lies inside courtyard outline"}));
                }
            }
        }
    }
    ClearanceReport {
        conflicts,
        courtyard_intersections,
        minimum,
    }
}

fn segment_bbox(points: &[(f64, f64)], margin: f64) -> (f64, f64, f64, f64) {
    let (mut min_x, mut max_x, mut min_y, mut max_y) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for &(x, y) in points {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    (
        min_x - margin,
        min_y - margin,
        max_x + margin,
        max_y + margin,
    )
}

fn bbox_intersects(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> bool {
    a.0 <= b.2 && a.2 >= b.0 && a.1 <= b.3 && a.3 >= b.1
}

fn polygon_intersects_route(polygon: &[IpcVector2], points: &[(f64, f64)], margin: f64) -> bool {
    if polygon.len() < 3 {
        return true;
    }
    let polygon_bbox = polygon.iter().fold(
        (
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ),
        |(min_x, min_y, max_x, max_y), p| {
            (
                min_x.min(p.x),
                min_y.min(p.y),
                max_x.max(p.x),
                max_y.max(p.y),
            )
        },
    );
    if !bbox_intersects(segment_bbox(points, margin), polygon_bbox) {
        return false;
    }
    points.iter().any(|p| point_in_polygon(*p, polygon))
        || points.windows(2).any(|route| {
            polygon
                .iter()
                .cycle()
                .take(polygon.len() + 1)
                .collect::<Vec<_>>()
                .windows(2)
                .any(|edge| {
                    segment_distance(
                        (route[0].0, route[0].1),
                        (route[1].0, route[1].1),
                        (edge[0].x, edge[0].y),
                        (edge[1].x, edge[1].y),
                    ) <= margin
                })
        })
}

fn candidate_unsupported_geometry(
    g: &IpcRoutingGeometry,
    points: &[(f64, f64)],
    layer: &str,
    width: f64,
    clearance: f64,
    net: &str,
) -> Vec<serde_json::Value> {
    let margin = width / 2.0 + clearance;
    let route_bbox = segment_bbox(points, margin);
    let mut encountered = Vec::new();

    for pad in g.pads.items.iter().filter(|pad| {
        !pad.geometry_supported
            && pad.net_name != net
            && (pad.layers.is_empty() || pad.layers.iter().any(|candidate| candidate == layer))
            && pad_intersects_segment_search_envelope(
                pad,
                points[0],
                *points.last().expect("route has two points"),
                margin,
            )
    }) {
        encountered.push(json!({
            "object_type": "unsupported_pad",
            "reference": pad.reference,
            "pad": pad.number,
            "shape": pad.shape,
            "reason": pad.geometry_reason.as_deref().unwrap_or("unsupported pad geometry")
        }));
    }

    if !g.track_arcs.available {
        encountered.push(json!({
            "object_type": "track_arc_geometry_unavailable",
            "layer": layer,
            "reason": g.track_arcs.reason.as_deref().unwrap_or("track arc geometry unavailable")
        }));
    } else {
        for arc in g
            .track_arcs
            .items
            .iter()
            .filter(|arc| arc.layer == layer && arc.net_name != net)
        {
            let arc_bbox = segment_bbox(
                &[
                    (arc.start.x, arc.start.y),
                    (arc.mid.x, arc.mid.y),
                    (arc.end.x, arc.end.y),
                ],
                arc.width / 2.0,
            );
            if bbox_intersects(route_bbox, arc_bbox) {
                encountered.push(json!({
                    "object_type": "unsupported_track_arc",
                    "layer": arc.layer,
                    "net": arc.net_name,
                    "reason": "track arc intersects candidate clearance envelope; exact arc clearance is unavailable"
                }));
            }
        }
    }

    if !g.zones.available {
        encountered.push(json!({
            "object_type": "zone_geometry_unavailable",
            "layer": layer,
            "reason": g.zones.reason.as_deref().unwrap_or("copper-zone geometry unavailable")
        }));
    } else {
        for zone in g.zones.items.iter().filter(|zone| {
            zone.layer == layer && zone.net_name.as_deref().unwrap_or_default() != net
        }) {
            let supported_outline =
                !zone.polygons.is_empty() && zone.polygons.iter().all(|polygon| polygon.len() >= 3);
            let relevant = zone
                .polygons
                .iter()
                .any(|polygon| polygon_intersects_route(polygon, points, margin));
            if relevant || !supported_outline {
                encountered.push(json!({
                    "object_type": "unsupported_foreign_zone",
                    "layer": zone.layer,
                    "zone_net": zone.net_name,
                    "reason": if relevant { "foreign zone fill/holes are unavailable in the candidate corridor" } else { "foreign zone outline is unavailable" }
                }));
            }
        }
    }

    if !g.board_edges.available {
        encountered.push(json!({
            "object_type": "unsupported_edge_cuts_geometry",
            "reason": g.board_edges.reason.as_deref().unwrap_or("Edge.Cuts geometry unavailable")
        }));
    }
    encountered
}

async fn routing_snapshot(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<Result<IpcRoutingGeometry, CallToolResult>> {
    let board = get_path(args, "board")?;
    let addr = ctx.config.ipc_address.clone();
    match with_ipc(addr, move |c| {
        c.ensure_board_is_active(&board)?;
        c.get_routing_geometry()
    })
    .await?
    {
        Ok(snapshot) => Ok(Ok(snapshot)),
        Err(msg) => Ok(Err(CallToolResult::error(format!(
            "KiCAD must be running with the board loaded (IPC error: {})",
            msg
        )))),
    }
}

async fn handle_get_routing_geometry(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let mut g = match routing_snapshot(args, ctx).await? {
        Ok(g) => g,
        Err(e) => return Ok(e),
    };
    let layer = args["layer"].as_str();
    let net = args["net"].as_str();
    let region = match parse_region(args) {
        Ok(region) => region,
        Err(error) => return Ok(error),
    };
    g.tracks.items.retain(|t| {
        layer.map(|v| t.layer == v).unwrap_or(true)
            && net.map(|v| t.net_name == v).unwrap_or(true)
            && (geometry_in_region(&t.start, region) || geometry_in_region(&t.end, region))
    });
    g.pads.items.retain(|p| {
        layer
            .map(|v| p.layers.is_empty() || p.layers.iter().any(|l| l == v))
            .unwrap_or(true)
            && net.map(|v| p.net_name == v).unwrap_or(true)
            && geometry_in_region(&p.position, region)
    });
    g.vias.items.retain(|v| {
        layer
            .map(|l| v.layers.is_empty() || v.layers.iter().any(|x| x == l))
            .unwrap_or(true)
            && net.map(|n| v.net_name == n).unwrap_or(true)
            && geometry_in_region(&v.position, region)
    });
    g.zones.items.retain(|z| {
        layer.map(|v| z.layer == v).unwrap_or(true)
            && net
                .map(|v| z.net_name.as_deref() == Some(v))
                .unwrap_or(true)
    });
    g.courtyards.items.retain(|c| {
        region
            .map(|r| {
                c.polygons
                    .iter()
                    .flatten()
                    .any(|p| geometry_in_region(p, Some(r)))
            })
            .unwrap_or(true)
    });
    Ok(CallToolResult::json(&json!({
        "source": "kicad_ipc", "read_only": true, "filters": {"layer":layer,"net":net,"region":region},
        "board_bounds": g.board_extents, "board_edges":g.board_edges, "footprints": g.footprints, "pads": g.pads,
        "tracks": g.tracks, "track_arcs": g.track_arcs, "vias": g.vias, "courtyards": g.courtyards, "zones": g.zones,
        "unsupported_or_unavailable": [
            "Edge.Cuts lines and arcs are exact IPC primitives; other Edge.Cuts primitive types are not emitted.",
            "Footprint body bounding boxes are unavailable; only IPC courtyard line/rectangle/polygon geometry is returned.",
            "Courtyard arcs/circles/beziers are not emitted because this tool does not approximate curves.",
            "Zone holes, arc nodes, and filled-copper polygons are not emitted; zone outline point nodes are returned.",
            "Track arcs are not included; tracks contains straight trace segments only."
        ]
    })))
}

async fn handle_check_route_clearance(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let points = match parse_points(args) {
        Ok(p) => p,
        Err(e) => return Ok(e),
    };
    let layer = match require_str(args, "layer") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let net = match require_str(args, "net") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let width = match require_f64(args, "trace_width") {
        Ok(v) if v > 0.0 => v,
        Ok(_) => return Ok(CallToolResult::error("trace_width must be positive")),
        Err(e) => return Ok(e),
    };
    let clearance = match require_f64(args, "required_clearance") {
        Ok(v) if v >= 0.0 => v,
        Ok(_) => {
            return Ok(CallToolResult::error(
                "required_clearance must be non-negative",
            ))
        }
        Err(e) => return Ok(e),
    };
    let allowed_endpoint_pads = match parse_allowed_endpoint_pads(args) {
        Ok(value) => value,
        Err(e) => return Ok(e),
    };
    let g = match routing_snapshot(args, ctx).await? {
        Ok(g) => g,
        Err(e) => return Ok(e),
    };
    let report = calculate_clearance(
        &g,
        &points,
        &layer,
        &net,
        width,
        clearance,
        &allowed_endpoint_pads,
    );
    let route_layer_supported =
        g.layers.available && g.layers.items.iter().any(|l| l.name == layer);
    let required_copper_supported =
        g.tracks.available && g.pads.available && g.vias.available && route_layer_supported;
    let board_edge_check = board_edge_check_status(&g);
    let encountered_unsupported_geometry =
        candidate_unsupported_geometry(&g, &points, &layer, width, clearance, &net);
    let board_edge_check_passed = g.board_edges.available
        && !report
            .conflicts
            .iter()
            .any(|conflict| conflict["object_type"] == "board_edge");
    let unsupported_capabilities = vec![
        json!("Custom, trapezoid, and chamfered-rectangle pad geometry is unsupported."),
        json!("Track arcs are reported but exact arc clearance is unsupported."),
        json!("Copper-zone filled polygons, holes, and fill-rule details are not modeled."),
        json!("Rule areas and KiCad design-rule/netclass overrides are not clearance-checked."),
        json!("Unsupported Edge.Cuts primitive types are not modeled."),
        json!("Ordinary courtyard curves and containment are not clearance-checked."),
    ];
    let collision_free = report.conflicts.is_empty();
    let globally_safe = collision_free
        && required_copper_supported
        && board_edge_check_passed
        && encountered_unsupported_geometry.is_empty();
    Ok(CallToolResult::json(&json!({
        "collision_free_against_checked_geometry": collision_free,
        "globally_safe": globally_safe,
        "required_copper_checks_supported": required_copper_supported,
        "board_edge_check": board_edge_check,
        "geometry_availability": {"layers":g.layers,"tracks":g.tracks,"track_arcs":g.track_arcs,"pads":g.pads,"vias":g.vias,"courtyards":g.courtyards,"zones":g.zones,"board_edges":g.board_edges,"board_bounds":g.board_extents},
        "read_only": true, "net":net,"layer":layer,"trace_width":width,"required_clearance":clearance,
        "minimum_clearance": if report.minimum.is_finite(){Some(report.minimum)}else{None},
        "collisions":report.conflicts,
        "courtyard_intersections":report.courtyard_intersections,
        "unsupported_capabilities": unsupported_capabilities,
        "encountered_unsupported_geometry": encountered_unsupported_geometry,
        "allowed_endpoint_pads":allowed_endpoint_pads.iter().map(|p| json!({"reference":p.reference,"pad":p.pad})).collect::<Vec<_>>(),
        "unsupported_checks": unsupported_capabilities.clone()
    })))
}

fn copper_layer_index(layer: &str) -> Option<usize> {
    if layer == "F.Cu" {
        Some(0)
    } else if layer == "B.Cu" {
        Some(33)
    } else {
        layer
            .strip_prefix("In")?
            .strip_suffix(".Cu")?
            .parse::<usize>()
            .ok()
    }
}

fn point_in_polygon(point: (f64, f64), polygon: &[IpcVector2]) -> bool {
    if polygon.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut previous = polygon.last().expect("polygon has at least three points");
    for current in polygon {
        let crosses = (current.y > point.1) != (previous.y > point.1)
            && point.0
                < (previous.x - current.x) * (point.1 - current.y) / (previous.y - current.y)
                    + current.x;
        if crosses {
            inside = !inside;
        }
        previous = current;
    }
    inside
}

fn foreign_zone_crossings(
    geometry: &IpcRoutingGeometry,
    layers: &[String],
    net: &str,
    point: (f64, f64),
    required_antipad_radius: f64,
) -> (Vec<serde_json::Value>, bool) {
    let mut reports = Vec::new();
    let mut all_supported = geometry.zones.available;
    for layer in layers {
        for zone in geometry.zones.items.iter().filter(|zone| {
            zone.layer == *layer && zone.net_name.as_deref().unwrap_or_default() != net
        }) {
            let outline_supported =
                !zone.polygons.is_empty() && zone.polygons.iter().all(|polygon| polygon.len() >= 3);
            let crosses = if outline_supported {
                zone.polygons
                    .iter()
                    .any(|polygon| point_in_polygon(point, polygon))
            } else {
                true
            };
            if !crosses {
                continue;
            }
            let nearby_geometry_supported = geometry.pads.available
                && geometry.tracks.available
                && geometry.vias.available
                && geometry.board_edges.available
                && geometry
                    .pads
                    .items
                    .iter()
                    .filter(|pad| pad.layers.is_empty() || pad.layers.contains(layer))
                    .all(|pad| pad.geometry_supported);
            let supported = outline_supported && nearby_geometry_supported;
            all_supported &= supported;
            reports.push(json!({
                "layer": layer,
                "zone_net": zone.net_name.as_deref().unwrap_or_default(),
                "antipad_required": true,
                "required_antipad_radius": required_antipad_radius,
                "supported": supported
            }));
        }
    }
    (reports, all_supported)
}

fn traversed_layers(
    enabled_layers: &[konnect_ipc::IpcLayer],
    start: &str,
    end: &str,
) -> Option<Vec<String>> {
    let mut copper: Vec<_> = enabled_layers
        .iter()
        .filter_map(|layer| {
            copper_layer_index(&layer.name).map(|index| (index, layer.name.clone()))
        })
        .collect();
    copper.sort_by_key(|(index, _)| *index);
    let start_index = copper.iter().position(|(_, name)| name == start)?;
    let end_index = copper.iter().position(|(_, name)| name == end)?;
    let (lo, hi) = (start_index.min(end_index), start_index.max(end_index));
    Some(
        copper[lo..=hi]
            .iter()
            .map(|(_, name)| name.clone())
            .collect(),
    )
}

async fn handle_check_via_clearance(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let number = |name| require_f64(args, name);
    let (x, y, diameter, drill, clearance) = match (
        number("x"),
        number("y"),
        number("diameter"),
        number("drill"),
        number("required_clearance"),
    ) {
        (Ok(a), Ok(b), Ok(c), Ok(d), Ok(e)) => (a, b, c, d, e),
        _ => {
            return Ok(CallToolResult::error(
                "x, y, diameter, drill, and required_clearance must be numeric",
            ))
        }
    };
    if diameter <= 0.0 || drill <= 0.0 || drill >= diameter || clearance < 0.0 {
        return Ok(CallToolResult::error(
            "require diameter > drill > 0 and non-negative required_clearance",
        ));
    }
    let (net, start, end) = match (
        require_str(args, "net"),
        require_str(args, "start_layer"),
        require_str(args, "end_layer"),
    ) {
        (Ok(a), Ok(b), Ok(c)) => (a.to_string(), b.to_string(), c.to_string()),
        _ => {
            return Ok(CallToolResult::error(
                "net, start_layer, and end_layer are required",
            ))
        }
    };
    let g = match routing_snapshot(args, ctx).await? {
        Ok(g) => g,
        Err(e) => return Ok(e),
    };
    if !g.layers.available {
        return Ok(CallToolResult::json(
            &json!({"read_only":true,"globally_safe":false,"all_required_geometry_supported":false,"reason":g.layers.reason.as_deref().unwrap_or("IPC board layers unavailable"),"traversed_layers":[]}),
        ));
    }
    let Some(layers) = traversed_layers(&g.layers.items, &start, &end) else {
        return Ok(CallToolResult::json(
            &json!({"read_only":true,"globally_safe":false,"all_required_geometry_supported":false,"reason":"start or end copper layer is not enabled on this board","start_layer":start,"end_layer":end,"traversed_layers":[]}),
        ));
    };
    let radius = diameter / 2.0;
    let mut collisions = Vec::new();
    let mut minimum = f64::INFINITY;
    let (zone_crossings, zone_crossings_supported) =
        foreign_zone_crossings(&g, &layers, &net, (x, y), radius + clearance);
    for layer in &layers {
        for p in g
            .pads
            .items
            .iter()
            .filter(|p| p.net_name != net && (p.layers.is_empty() || p.layers.contains(layer)))
        {
            let Some(d) = pad_segment_distance(p, (x, y), (x, y)) else {
                collisions.push(json!({"object_type":"pad_geometry_unavailable","layer":layer,"reference":p.reference,"pad":p.number,"reason":p.geometry_reason}));
                continue;
            };
            let gap = d - radius;
            minimum = minimum.min(gap);
            if gap < clearance {
                collisions.push(json!({"object_type":"pad","layer":layer,"reference":p.reference,"pad":p.number,"net":p.net_name,"actual_clearance":gap}));
            }
        }
        for t in g
            .tracks
            .items
            .iter()
            .filter(|t| t.layer == *layer && t.net_name != net)
        {
            let gap = point_segment_distance((x, y), (t.start.x, t.start.y), (t.end.x, t.end.y))
                - radius
                - t.width / 2.0;
            minimum = minimum.min(gap);
            if gap < clearance {
                collisions.push(json!({"object_type":"track","layer":layer,"net":t.net_name,"actual_clearance":gap}));
            }
        }
        for v in g
            .vias
            .items
            .iter()
            .filter(|v| v.net_name != net && (v.layers.is_empty() || v.layers.contains(layer)))
        {
            let vr = v.size.as_ref().map(|s| s.x.max(s.y) / 2.0);
            if let Some(vr) = vr {
                let gap =
                    ((x - v.position.x).powi(2) + (y - v.position.y).powi(2)).sqrt() - radius - vr;
                minimum = minimum.min(gap);
                if gap < clearance {
                    collisions.push(json!({"object_type":"via","layer":layer,"net":v.net_name,"actual_clearance":gap}));
                }
            } else {
                collisions.push(json!({"object_type":"via_geometry_unavailable","layer":layer,"net":v.net_name}));
            }
        }
    }
    for edge in &g.board_edges.items {
        let d = match edge {
            IpcBoardEdgeGeometry::Line { start, end } => {
                point_segment_distance((x, y), (start.x, start.y), (end.x, end.y))
            }
            IpcBoardEdgeGeometry::Arc { start, mid, end } => segment_arc_distance(
                (x, y),
                (x, y),
                (start.x, start.y),
                (mid.x, mid.y),
                (end.x, end.y),
            ),
        };
        let gap = d - radius;
        minimum = minimum.min(gap);
        if gap < clearance {
            collisions
                .push(json!({"object_type":"board_edge","geometry":edge,"actual_clearance":gap}));
        }
    }
    let supported = g.tracks.available
        && g.pads.available
        && g.pads
            .items
            .iter()
            .filter(|pad| {
                pad.layers.is_empty() || pad.layers.iter().any(|layer| layers.contains(layer))
            })
            .all(|p| p.geometry_supported)
        && g.vias.available
        && g.board_edges.available
        && g.zones.available
        && zone_crossings_supported;
    Ok(CallToolResult::json(
        &json!({"read_only":true,"globally_safe":supported&&collisions.is_empty(),"collision_free_against_checked_geometry":collisions.is_empty(),"all_required_geometry_supported":supported,"position":{"x":x,"y":y},"net":net,"diameter":diameter,"drill":drill,"start_layer":start,"end_layer":end,"traversed_layers":layers,"required_clearance":clearance,"minimum_clearance":if minimum.is_finite(){Some(minimum)}else{None},"collisions":collisions,"zone_crossings":zone_crossings,"same_net_contact":"legal and excluded from collisions","filled_zone_modeling":"Zone outlines identify plane crossings; filled polygons are not claimed as fully modeled. Foreign-plane antipads are conservatively required and nearby supported copper/edge geometry is checked."}),
    ))
}

async fn handle_add_net(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };

    let content = std::fs::read_to_string(&board_path)?;
    // Count existing nets to determine next net ID
    let net_id = content.matches("(net ").count() as i32;
    let net_sexp = format!("\n  (net {net_id} \"{net_name}\")");
    // Insert before the last closing paren
    let close_pos = content.rfind(')').unwrap_or(content.len());
    let new_content = apply_edits(content, vec![SexpEdit::insert(close_pos, net_sexp)]);
    write_atomic(&board_path, &new_content)?;

    Ok(CallToolResult::json(
        &json!({ "net_id": net_id, "net_name": net_name }),
    ))
}

async fn handle_route_trace(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer = match require_str(args, "layer") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x1 = match require_f64(args, "x1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y1 = match require_f64(args, "y1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let x2 = match require_f64(args, "x2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y2 = match require_f64(args, "y2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let width = args["width"].as_f64().unwrap_or(0.25);

    let net_ipc = net_name.clone();
    let layer_ipc = layer.clone();
    ipc!(ctx, |c| c
        .add_track(&net_ipc, &layer_ipc, width, x1, y1, x2, y2));
    Ok(CallToolResult::json(&json!({
        "net": net_name, "layer": layer, "width": width,
        "from": { "x": x1, "y": y1 }, "to": { "x": x2, "y": y2 }
    })))
}

async fn handle_route_pad_to_pad(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let ref1 = match require_str(args, "ref1") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let pad1 = match require_str(args, "pad1") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let ref2 = match require_str(args, "ref2") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let pad2 = match require_str(args, "pad2") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer = args["layer"].as_str().unwrap_or("F.Cu").to_string();
    let width = args["width"].as_f64().unwrap_or(0.25);

    // Look up pad positions from the PCB S-expression file
    let content = std::fs::read_to_string(&board_path)?;
    let tree = konnect_sexp::parser::parse_sexp(&content)?;

    let pos1 = find_pad_board_position(&tree, &ref1, &pad1)?;
    let pos2 = find_pad_board_position(&tree, &ref2, &pad2)?;

    // Route an L-bend: horizontal first, then vertical
    let (x1, y1) = pos1;
    let (x2, y2) = pos2;
    let net_ipc = net_name.clone();
    let layer_ipc = layer.clone();

    if (x1 - x2).abs() < 0.01 || (y1 - y2).abs() < 0.01 {
        // Already axis-aligned: single segment
        ipc!(ctx, |c| c
            .add_track(&net_ipc, &layer_ipc, width, x1, y1, x2, y2));
    } else {
        // L-bend: horizontal then vertical
        let mid_x = x2;
        let mid_y = y1;
        let net_a = net_name.clone();
        let net_b = net_name.clone();
        let layer_a = layer.clone();
        let layer_b = layer.clone();
        ipc!(ctx, |c| {
            c.add_track(&net_a, &layer_a, width, x1, y1, mid_x, mid_y)?;
            c.add_track(&net_b, &layer_b, width, mid_x, mid_y, x2, y2)?;
            Ok(())
        });
    }

    Ok(CallToolResult::json(&json!({
        "routed": true,
        "net": net_name, "layer": layer, "width": width,
        "from": { "ref": ref1, "pad": pad1, "x": x1, "y": y1 },
        "to":   { "ref": ref2, "pad": pad2, "x": x2, "y": y2 }
    })))
}

/// Look up a pad's board-space (x, y) position from the parsed PCB S-expression tree.
fn find_pad_board_position(
    tree: &konnect_sexp::parser::SexpNode,
    reference: &str,
    pad_number: &str,
) -> anyhow::Result<(f64, f64)> {
    let fp_node = tree
        .find_all("footprint")
        .into_iter()
        .find(|fp| {
            fp.find_all("property").iter().any(|p| {
                p.get(1).and_then(|n| n.as_str()) == Some("Reference")
                    && p.get(2).and_then(|n| n.as_str()) == Some(reference)
            })
        })
        .ok_or_else(|| anyhow::anyhow!("Footprint '{}' not found on board", reference))?;

    let fp_at = fp_node.find("at");
    let fp_x = fp_at.and_then(|a| a.get_f64(1)).unwrap_or(0.0);
    let fp_y = fp_at.and_then(|a| a.get_f64(2)).unwrap_or(0.0);
    let fp_rot = fp_at.and_then(|a| a.get_f64(3)).unwrap_or(0.0);

    let pad = fp_node
        .find_all("pad")
        .into_iter()
        .find(|p| p.get(1).and_then(|n| n.as_str()) == Some(pad_number))
        .ok_or_else(|| anyhow::anyhow!("Pad '{}' not found on '{}'", pad_number, reference))?;

    let pad_at = pad
        .find("at")
        .ok_or_else(|| anyhow::anyhow!("Pad has no (at) node"))?;
    let local_x = pad_at.get_f64(1).unwrap_or(0.0);
    let local_y = pad_at.get_f64(2).unwrap_or(0.0);

    // Transform local pad coords to board space (rotation only).
    // Uses the canonical KiCAD transform — see konnect_sexp::geometry.
    Ok(konnect_sexp::geometry::transform_pad(
        local_x, local_y, fp_x, fp_y, fp_rot,
    ))
}

async fn handle_add_via(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x = match require_f64(args, "x") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y = match require_f64(args, "y") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let drill = args["drill"].as_f64().unwrap_or(0.4);
    let pad_size = args["pad_size"].as_f64().unwrap_or(0.8);

    let net_ipc = net_name.clone();
    ipc!(ctx, |c| c.add_via(&net_ipc, x, y, drill, pad_size));
    Ok(CallToolResult::json(
        &json!({ "net": net_name, "x": x, "y": y, "drill": drill, "pad_size": pad_size }),
    ))
}

async fn handle_add_copper_pour(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer = match require_str(args, "layer") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let clearance = args["clearance"].as_f64().unwrap_or(0.2);
    let min_w = args["min_width"].as_f64().unwrap_or(0.25);
    let pts_arr = match args["points"].as_array() {
        Some(a) => a.clone(),
        None => return Ok(CallToolResult::error("Missing 'points' array")),
    };

    let pts: Vec<(f64, f64)> = pts_arr
        .iter()
        .filter_map(|p| Some((p["x"].as_f64()?, p["y"].as_f64()?)))
        .collect();
    if pts.len() < 3 {
        return Ok(CallToolResult::error("Zone requires at least 3 points"));
    }

    let content = std::fs::read_to_string(&board_path)?;
    let net_id = find_net_id(&content, &net_name);
    let zone_s = format_zone(net_id, &net_name, &layer, clearance, min_w, &pts);
    let close = content.rfind(')').unwrap_or(content.len());
    let new_content = apply_edits(content, vec![SexpEdit::insert(close, zone_s)]);
    write_atomic(&board_path, &new_content)?;

    Ok(CallToolResult::json(
        &json!({ "net": net_name, "layer": layer, "points": pts.len() }),
    ))
}

async fn handle_delete_trace(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let uuid = match require_str(args, "uuid") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };

    let uuid_ipc = uuid.clone();
    ipc!(ctx, |c| c.delete_track(&uuid_ipc));
    Ok(CallToolResult::json(&json!({ "deleted_uuid": uuid })))
}

async fn handle_query_traces(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let net = args["net_name"].as_str().map(String::from);
    let layer = args["layer"].as_str().map(String::from);

    let tracks = ipc!(ctx, |c| { c.get_tracks(net.as_deref(), layer.as_deref()) });

    let items: Vec<serde_json::Value> = tracks
        .iter()
        .map(|t| {
            json!({
                "net": t.net_name, "layer": t.layer, "width": t.width,
                "x1": t.start.x, "y1": t.start.y,
                "x2": t.end.x,   "y2": t.end.y
            })
        })
        .collect();

    Ok(CallToolResult::json(
        &json!({ "count": items.len(), "traces": items }),
    ))
}

async fn handle_get_nets_list(
    _args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let nets = ipc!(ctx, |c| c.get_nets());
    let items: Vec<serde_json::Value> = nets
        .iter()
        .map(|n| json!({ "name": n.name, "netcode": n.netcode }))
        .collect();
    Ok(CallToolResult::json(
        &json!({ "count": items.len(), "nets": items }),
    ))
}

async fn handle_modify_trace(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let uuid = match require_str(args, "uuid") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer = match require_str(args, "layer") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let x1 = match require_f64(args, "x1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y1 = match require_f64(args, "y1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let x2 = match require_f64(args, "x2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y2 = match require_f64(args, "y2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let width = args["width"].as_f64().unwrap_or(0.25);

    let uuid_ipc = uuid.clone();
    let net_ipc = net_name.clone();
    let layer_ipc = layer.clone();
    ipc!(ctx, |c| {
        c.delete_track(&uuid_ipc)?;
        c.add_track(&net_ipc, &layer_ipc, width, x1, y1, x2, y2)
    });
    Ok(CallToolResult::json(&json!({
        "modified_uuid": uuid,
        "net": net_name, "layer": layer, "width": width,
        "from": { "x": x1, "y": y1 }, "to": { "x": x2, "y": y2 }
    })))
}

async fn handle_create_netclass(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let name = match require_str(args, "name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let clearance = args["clearance"].as_f64().unwrap_or(0.2);
    let trace_width = args["trace_width"].as_f64().unwrap_or(0.25);
    let via_drill = args["via_drill"].as_f64().unwrap_or(0.4);
    let via_dia = args["via_diameter"].as_f64().unwrap_or(0.8);

    let netclass_sexp = format!(
        "\n      (netclass \"{name}\"\n        (clearance {clearance})\n        \
         (trace_width {trace_width})\n        (via_drill {via_drill})\n        \
         (via_diameter {via_dia})\n      )"
    );

    let content = std::fs::read_to_string(&board_path)?;
    // Find (net_classes block or (net_settings block to insert into
    let insert_pos = if let Some(nc_pos) = content.find("(net_classes") {
        // Find closing paren of (net_classes ...)
        let block = &content[nc_pos..];
        nc_pos
            + block
                .find("\n    )")
                .unwrap_or(block.find(')').unwrap_or(block.len() - 1))
    } else {
        // No net_classes block; insert before last )
        content.rfind(')').unwrap_or(content.len())
    };

    let new_content = apply_edits(content, vec![SexpEdit::insert(insert_pos, netclass_sexp)]);
    write_atomic(&board_path, &new_content)?;

    Ok(CallToolResult::json(&json!({
        "created_netclass": name,
        "clearance": clearance, "trace_width": trace_width,
        "via_drill": via_drill, "via_diameter": via_dia
    })))
}

async fn handle_assign_net_to_class(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let board_path = get_path(args, "board")?;
    let net_name = match require_str(args, "net_name") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let netclass = match require_str(args, "netclass") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };

    let content = std::fs::read_to_string(&board_path)?;

    // Find the netclass block: (netclass "NAME" ...)
    let nc_pat = format!("(netclass \"{}\"", netclass);
    let nc_pos = match content.find(&nc_pat) {
        Some(p) => p,
        None => {
            return Ok(CallToolResult::error(format!(
                "Netclass '{}' not found in board file",
                netclass
            )))
        }
    };

    // Find the closing paren of the netclass block
    let mut depth = 0i32;
    let mut nc_end = nc_pos;
    for (i, ch) in content[nc_pos..].char_indices() {
        match ch {
            '(' => depth += 1,
            ')' => {
                depth -= 1;
                if depth == 0 {
                    nc_end = nc_pos + i;
                    break;
                }
            }
            _ => {}
        }
    }

    // Check if net is already assigned
    let nc_block = &content[nc_pos..nc_end];
    let net_check = format!("(net \"{}\")", net_name);
    if nc_block.contains(&net_check) {
        return Ok(CallToolResult::json(&json!({
            "already_assigned": true,
            "net_name": net_name,
            "netclass": netclass
        })));
    }

    // Insert the net assignment before the closing paren of the netclass block
    let net_entry = format!("\n        (net \"{}\")", net_name);
    let new_content = apply_edits(content, vec![SexpEdit::insert(nc_end, net_entry)]);
    write_atomic(&board_path, &new_content)?;

    Ok(CallToolResult::json(&json!({
        "assigned": true,
        "net_name": net_name,
        "netclass": netclass
    })))
}

async fn handle_route_diff_pair(
    args: &serde_json::Value,
    ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let net_pos = match require_str(args, "net_pos") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let net_neg = match require_str(args, "net_neg") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let layer = args["layer"].as_str().unwrap_or("F.Cu").to_string();
    let x1 = match require_f64(args, "x1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y1 = match require_f64(args, "y1") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let x2 = match require_f64(args, "x2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let y2 = match require_f64(args, "y2") {
        Ok(v) => v,
        Err(e) => return Ok(e),
    };
    let width = args["width"].as_f64().unwrap_or(0.1);
    let gap = args["gap"].as_f64().unwrap_or(0.1);
    let offset = (gap + width) / 2.0;

    // Route two parallel traces offset perpendicular to the direction
    let dx = x2 - x1;
    let dy = y2 - y1;
    let len = (dx * dx + dy * dy).sqrt().max(1e-9);
    let perp_x = -dy / len * offset;
    let perp_y = dx / len * offset;

    let np_ipc = net_pos.clone();
    let nn_ipc = net_neg.clone();
    let layer_ipc = layer.clone();
    ipc!(ctx, |c| {
        c.add_track(
            &np_ipc,
            &layer_ipc,
            width,
            x1 + perp_x,
            y1 + perp_y,
            x2 + perp_x,
            y2 + perp_y,
        )?;
        c.add_track(
            &nn_ipc,
            &layer_ipc,
            width,
            x1 - perp_x,
            y1 - perp_y,
            x2 - perp_x,
            y2 - perp_y,
        )
    });

    Ok(CallToolResult::json(&json!({
        "net_pos": net_pos, "net_neg": net_neg,
        "layer": layer, "width": width, "gap": gap
    })))
}

#[cfg(test)]
mod routing_inspection_tests {
    use super::*;
    use konnect_ipc::{
        IpcGeometryClass, IpcPadGeometry, IpcPolygonGeometry, IpcTrack, IpcTrackArc, IpcViaGeometry,
    };

    fn pad(reference: &str, number: &str, net: &str, x: f64, y: f64) -> IpcPadGeometry {
        IpcPadGeometry {
            reference: reference.into(),
            number: number.into(),
            position: IpcVector2 { x, y },
            size: Some(IpcVector2 { x: 0.6, y: 0.6 }),
            shape: Some("circle".into()),
            rotation: 0.0,
            corner_rounding_ratio: None,
            geometry_supported: true,
            geometry_reason: None,
            layers: vec!["F.Cu".into()],
            net_name: net.into(),
            net_id: 1,
        }
    }

    fn empty_geometry() -> IpcRoutingGeometry {
        IpcRoutingGeometry {
            layers: IpcGeometryClass::available(vec![
                konnect_ipc::IpcLayer {
                    name: "F.Cu".into(),
                    id: 0,
                    kind: "copper".into(),
                },
                konnect_ipc::IpcLayer {
                    name: "In1.Cu".into(),
                    id: 1,
                    kind: "copper".into(),
                },
                konnect_ipc::IpcLayer {
                    name: "In2.Cu".into(),
                    id: 2,
                    kind: "copper".into(),
                },
                konnect_ipc::IpcLayer {
                    name: "B.Cu".into(),
                    id: 31,
                    kind: "copper".into(),
                },
            ]),
            board_extents: IpcGeometryClass::unavailable("No bounding box returned from KiCAD"),
            footprints: IpcGeometryClass::available(vec![]),
            pads: IpcGeometryClass::available(vec![]),
            tracks: IpcGeometryClass::available(vec![]),
            track_arcs: IpcGeometryClass::available(vec![]),
            vias: IpcGeometryClass::available(vec![]),
            courtyards: IpcGeometryClass::available(vec![]),
            zones: IpcGeometryClass::available(vec![]),
            board_edges: IpcGeometryClass::available(vec![]),
        }
    }

    fn geometry_without_bounds() -> IpcRoutingGeometry {
        let mut geometry = empty_geometry();
        geometry.tracks = IpcGeometryClass::available(vec![IpcTrack {
            net_name: "GND".into(),
            layer: "F.Cu".into(),
            width: 0.25,
            start: IpcVector2 { x: 0.0, y: 1.0 },
            end: IpcVector2 { x: 2.0, y: 1.0 },
        }]);
        geometry
    }

    fn allowed(reference: &str, pad: &str) -> Vec<AllowedEndpointPad> {
        vec![AllowedEndpointPad {
            reference: reference.into(),
            pad: pad.into(),
        }]
    }

    fn route_safe(geometry: &IpcRoutingGeometry, points: &[(f64, f64)], layer: &str) -> bool {
        let report = calculate_clearance(geometry, points, layer, "SIGNAL", 0.2, 0.2, &[]);
        let encountered =
            candidate_unsupported_geometry(geometry, points, layer, 0.2, 0.2, "SIGNAL");
        report.conflicts.is_empty()
            && geometry.layers.available
            && geometry.layers.items.iter().any(|item| item.name == layer)
            && geometry.tracks.available
            && geometry.pads.available
            && geometry.vias.available
            && geometry.board_edges.available
            && encountered.is_empty()
    }

    #[test]
    fn clean_route_is_globally_safe() {
        assert!(route_safe(
            &empty_geometry(),
            &[(1.0, 1.0), (3.0, 1.0)],
            "F.Cu"
        ));
    }

    #[test]
    fn unsupported_custom_pad_is_capability_only_when_far_away() {
        let mut geometry = empty_geometry();
        let mut custom = pad("U1", "1", "OTHER", 20.0, 20.0);
        custom.shape = Some("PssCustom".into());
        custom.geometry_supported = false;
        custom.geometry_reason = Some("unsupported IPC pad shape: PssCustom".into());
        geometry.pads.items.push(custom);
        assert!(route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
        assert!(!geometry.pads.items[0].geometry_supported);
    }

    #[test]
    fn unsupported_custom_pad_in_corridor_fails_closed() {
        let mut geometry = empty_geometry();
        let mut custom = pad("U1", "1", "OTHER", 2.0, 1.0);
        custom.shape = Some("PssCustom".into());
        custom.geometry_supported = false;
        geometry.pads.items.push(custom);
        assert!(!route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
        assert_eq!(
            candidate_unsupported_geometry(
                &geometry,
                &[(1.0, 1.0), (3.0, 1.0)],
                "F.Cu",
                0.2,
                0.2,
                "SIGNAL"
            )
            .len(),
            1
        );
    }

    fn track_arc(net: &str, layer: &str, x: f64, y: f64) -> IpcTrackArc {
        IpcTrackArc {
            net_name: net.into(),
            layer: layer.into(),
            width: 0.2,
            start: IpcVector2 { x, y },
            mid: IpcVector2 {
                x: x + 0.5,
                y: y + 0.5,
            },
            end: IpcVector2 { x: x + 1.0, y },
        }
    }

    #[test]
    fn unrelated_track_arc_does_not_fail_route() {
        let mut geometry = empty_geometry();
        geometry
            .track_arcs
            .items
            .push(track_arc("OTHER", "F.Cu", 20.0, 20.0));
        assert!(route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
    }

    #[test]
    fn intersecting_track_arc_fails_closed() {
        let mut geometry = empty_geometry();
        geometry
            .track_arcs
            .items
            .push(track_arc("OTHER", "F.Cu", 1.5, 1.0));
        assert!(!route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
    }

    #[test]
    fn foreign_zone_on_unrelated_layer_does_not_fail_route() {
        let mut geometry = empty_geometry();
        geometry.zones.items.push(IpcPolygonGeometry {
            object_type: "zone_outline".into(),
            reference: None,
            layer: "In1.Cu".into(),
            net_name: Some("GND".into()),
            net_id: Some(1),
            polygons: vec![vec![
                IpcVector2 { x: 0.0, y: 0.0 },
                IpcVector2 { x: 0.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 0.0 },
            ]],
        });
        assert!(route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
    }

    #[test]
    fn relevant_foreign_zone_fails_closed() {
        let mut geometry = empty_geometry();
        geometry.zones.items.push(IpcPolygonGeometry {
            object_type: "zone_outline".into(),
            reference: None,
            layer: "F.Cu".into(),
            net_name: Some("GND".into()),
            net_id: Some(1),
            polygons: vec![vec![
                IpcVector2 { x: 0.0, y: 0.0 },
                IpcVector2 { x: 0.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 0.0 },
            ]],
        });
        assert!(!route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
    }

    #[test]
    fn ordinary_courtyard_intersection_is_informational() {
        let mut geometry = empty_geometry();
        geometry.courtyards.items.push(IpcPolygonGeometry {
            object_type: "courtyard".into(),
            reference: Some("U1".into()),
            layer: "F.CrtYd".into(),
            net_name: None,
            net_id: None,
            polygons: vec![vec![
                IpcVector2 { x: 0.0, y: 0.0 },
                IpcVector2 { x: 0.0, y: 2.0 },
                IpcVector2 { x: 3.0, y: 2.0 },
                IpcVector2 { x: 3.0, y: 0.0 },
            ]],
        });
        let report = calculate_clearance(
            &geometry,
            &[(1.0, 1.0), (2.0, 1.0)],
            "F.Cu",
            "SIGNAL",
            0.2,
            0.2,
            &[],
        );
        assert!(!report.courtyard_intersections.is_empty());
        assert!(route_safe(&geometry, &[(1.0, 1.0), (2.0, 1.0)], "F.Cu"));
    }

    #[test]
    fn supported_foreign_track_and_pad_collisions_fail() {
        let mut track_geometry = empty_geometry();
        track_geometry.tracks.items.push(IpcTrack {
            net_name: "OTHER".into(),
            layer: "F.Cu".into(),
            width: 0.2,
            start: IpcVector2 { x: 1.0, y: 1.0 },
            end: IpcVector2 { x: 3.0, y: 1.0 },
        });
        assert!(!route_safe(
            &track_geometry,
            &[(1.0, 1.0), (3.0, 1.0)],
            "F.Cu"
        ));

        let mut pad_geometry = empty_geometry();
        pad_geometry
            .pads
            .items
            .push(pad("R1", "1", "OTHER", 2.0, 1.0));
        assert!(!route_safe(
            &pad_geometry,
            &[(1.0, 1.0), (3.0, 1.0)],
            "F.Cu"
        ));
    }

    #[test]
    fn exact_edge_clearance_passes_and_violation_fails() {
        let mut geometry = empty_geometry();
        geometry.board_edges.items.push(IpcBoardEdgeGeometry::Line {
            start: IpcVector2 { x: 0.0, y: 0.0 },
            end: IpcVector2 { x: 10.0, y: 0.0 },
        });
        assert!(route_safe(&geometry, &[(1.0, 1.0), (3.0, 1.0)], "F.Cu"));
        assert!(!route_safe(&geometry, &[(1.0, 0.1), (3.0, 0.1)], "F.Cu"));
    }

    #[test]
    fn endpoint_pad_exemption_does_not_exempt_neighbor() {
        let mut geometry = empty_geometry();
        geometry.pads.items.push(pad("J1", "1", "SIGNAL", 1.0, 1.0));
        geometry.pads.items.push(pad("R1", "1", "OTHER", 1.5, 1.0));
        let report = calculate_clearance(
            &geometry,
            &[(1.0, 1.0), (3.0, 1.0)],
            "F.Cu",
            "SIGNAL",
            0.2,
            0.2,
            &allowed("J1", "1"),
        );
        assert!(report
            .conflicts
            .iter()
            .any(|conflict| conflict["reference"] == "R1"));
        assert!(!report
            .conflicts
            .iter()
            .any(|conflict| conflict["reference"] == "J1"));
    }

    #[test]
    fn declared_start_endpoint_pad_is_excluded_but_neighbor_is_checked() {
        let mut geometry = empty_geometry();
        geometry.pads.items = vec![
            pad("U1", "1", "PWRKEY", 0.0, 0.0),
            pad("U1", "2", "GND", 0.0, 0.55),
        ];
        let report = calculate_clearance(
            &geometry,
            &[(0.0, 0.0), (1.0, 0.0)],
            "F.Cu",
            "ROUTE_NET",
            0.2,
            0.2,
            &allowed("U1", "1"),
        );
        assert!(!report.conflicts.iter().any(|c| c["pad"] == "1"));
        assert!(report.conflicts.iter().any(|c| c["pad"] == "2"));
    }

    #[test]
    fn declared_destination_endpoint_pad_is_excluded() {
        let mut geometry = empty_geometry();
        geometry.pads.items = vec![pad("Q2", "3", "PWRKEY", 1.0, 0.0)];
        let report = calculate_clearance(
            &geometry,
            &[(0.0, 0.0), (1.0, 0.0)],
            "F.Cu",
            "ROUTE_NET",
            0.2,
            0.2,
            &allowed("Q2", "3"),
        );
        assert!(report.conflicts.is_empty());
    }

    #[test]
    fn allowed_pad_crossed_by_middle_segment_still_collides() {
        let mut geometry = empty_geometry();
        geometry.pads.items = vec![pad("U1", "1", "PWRKEY", 1.0, 0.0)];
        let report = calculate_clearance(
            &geometry,
            &[(-1.0, 1.0), (0.0, 1.0), (2.0, 0.0), (3.0, 0.0)],
            "F.Cu",
            "ROUTE_NET",
            0.2,
            0.2,
            &allowed("U1", "1"),
        );
        assert!(report.conflicts.iter().any(|c| c["pad"] == "1"));
    }

    #[test]
    fn courtyard_crossing_is_informational_only() {
        let mut geometry = empty_geometry();
        geometry.courtyards.items = vec![IpcPolygonGeometry {
            object_type: "courtyard".into(),
            reference: Some("U10".into()),
            layer: "F.CrtYd".into(),
            net_name: None,
            net_id: None,
            polygons: vec![vec![
                IpcVector2 { x: 0.5, y: -1.0 },
                IpcVector2 { x: 0.5, y: 1.0 },
            ]],
        }];
        let report = calculate_clearance(
            &geometry,
            &[(0.0, 0.0), (1.0, 0.0)],
            "F.Cu",
            "SIGNAL",
            0.2,
            0.2,
            &[],
        );
        assert!(report.conflicts.is_empty());
        assert_eq!(report.courtyard_intersections.len(), 1);
    }

    #[test]
    fn foreign_track_crossing_still_collides() {
        let geometry = geometry_without_bounds();
        let report = calculate_clearance(
            &geometry,
            &[(1.0, 0.0), (1.0, 2.0)],
            "F.Cu",
            "MODEM_RESET_N",
            0.2,
            0.2,
            &[],
        );
        assert!(report.conflicts.iter().any(|c| c["object_type"] == "track"));
    }

    #[test]
    fn foreign_via_still_collides() {
        let mut geometry = empty_geometry();
        geometry.vias.items = vec![IpcViaGeometry {
            position: IpcVector2 { x: 0.5, y: 0.0 },
            size: Some(IpcVector2 { x: 0.8, y: 0.8 }),
            drill: Some(IpcVector2 { x: 0.4, y: 0.4 }),
            layers: vec!["F.Cu".into(), "B.Cu".into()],
            net_name: "GND".into(),
            net_id: 2,
        }];
        let report = calculate_clearance(
            &geometry,
            &[(0.0, 0.0), (1.0, 0.0)],
            "F.Cu",
            "SIGNAL",
            0.2,
            0.2,
            &[],
        );
        assert!(report.conflicts.iter().any(|c| c["object_type"] == "via"));
    }

    #[test]
    fn segment_distance_detects_crossing_and_separation() {
        assert_eq!(
            segment_distance((0.0, 0.0), (2.0, 0.0), (1.0, -1.0), (1.0, 1.0)),
            0.0
        );
        assert!(
            (segment_distance((0.0, 0.0), (2.0, 0.0), (0.0, 1.0), (2.0, 1.0)) - 1.0).abs() < 1e-9
        );
        assert_eq!(
            segment_distance((5.0, 12.1), (6.0, 12.1), (29.0, 12.1), (31.0, 12.1)),
            23.0
        );
    }

    #[test]
    fn distant_collinear_pad_cannot_collide_with_short_segment() {
        let mut geometry = empty_geometry();
        geometry.pads.items = vec![pad("U1", "51", "VBAT_ADC_SENSE", 30.0, 12.1)];
        let report = calculate_clearance(
            &geometry,
            &[(6.0, 12.1), (5.0, 12.1)],
            "F.Cu",
            "PWRKEY",
            0.2,
            0.2,
            &[],
        );
        assert!(report.conflicts.is_empty());
        assert!(!pad_intersects_segment_search_envelope(
            &geometry.pads.items[0],
            (6.0, 12.1),
            (5.0, 12.1),
            0.3
        ));
    }

    #[test]
    fn rotated_distant_pad_is_rejected_by_spatial_guard() {
        let mut distant = pad("U1", "36", "OTHER", 30.0, 27.1);
        distant.shape = Some("PssRectangle".into());
        distant.size = Some(IpcVector2 { x: 2.0, y: 0.6 });
        distant.rotation = 90.0;
        assert!(!pad_intersects_segment_search_envelope(
            &distant,
            (6.0, 27.1),
            (5.0, 27.1),
            0.3
        ));
        assert!(pad_segment_distance(&distant, (6.0, 27.1), (5.0, 27.1)).unwrap() > 20.0);
    }

    #[test]
    fn true_nearby_pad_collision_still_fails() {
        let mut geometry = empty_geometry();
        geometry.pads.items = vec![pad("R2", "2", "GND", 1.2, 0.4)];
        let report = calculate_clearance(
            &geometry,
            &[(0.0, 0.0), (2.0, 0.0)],
            "F.Cu",
            "WDT_DELAY",
            0.2,
            0.2,
            &[],
        );
        assert!(report
            .conflicts
            .iter()
            .any(|conflict| conflict["reference"] == "R2"));
    }

    #[test]
    fn proposed_points_validation_is_strict() {
        assert!(parse_points(&json!({"points":[{"x":0.0,"y":0.0},{"x":1.0,"y":1.0}]})).is_ok());
        assert!(parse_points(&json!({"points":[{"x":0.0,"y":0.0}]})).is_err());
        assert!(parse_points(&json!({"points":[{"x":0.0},{"x":1.0,"y":1.0}]})).is_err());
    }

    #[test]
    fn missing_board_bounds_preserves_routing_geometry() {
        let geometry = geometry_without_bounds();
        assert!(!geometry.board_extents.available);
        assert!(geometry.tracks.available);
        assert_eq!(geometry.tracks.items.len(), 1);
    }

    #[test]
    fn region_validation_is_independent_of_board_bounds() {
        assert_eq!(
            parse_region(&json!({"region":{"x_min":0.0,"y_min":1.0,"x_max":2.0,"y_max":3.0}}))
                .unwrap(),
            Some((0.0, 1.0, 2.0, 3.0))
        );
        assert!(
            parse_region(&json!({"region":{"x_min":2.0,"y_min":1.0,"x_max":2.0,"y_max":3.0}}))
                .is_err()
        );
        assert!(parse_region(
            &json!({"region":{"x_min":0.0,"y_min":1.0,"x_max":null,"y_max":3.0}})
        )
        .is_err());
    }

    #[test]
    fn trace_collision_is_detected_without_board_bounds() {
        let geometry = geometry_without_bounds();
        let track = &geometry.tracks.items[0];
        let center = segment_distance(
            (1.0, 0.0),
            (1.0, 2.0),
            (track.start.x, track.start.y),
            (track.end.x, track.end.y),
        );
        let gap = center - (0.25 + track.width) / 2.0;
        assert!(gap < 0.2);
    }

    #[test]
    fn board_edge_check_is_unsupported_when_ipc_edges_unavailable() {
        let mut geometry = geometry_without_bounds();
        geometry.board_edges = IpcGeometryClass::unavailable("IPC Edge.Cuts query failed");
        let status = board_edge_check_status(&geometry);
        assert_eq!(status["supported"], false);
        assert!(status["reason"].as_str().unwrap().contains("Edge.Cuts"));
    }

    #[test]
    fn unavailable_geometry_differs_from_valid_empty_geometry() {
        let unavailable: IpcGeometryClass<Vec<IpcTrack>> =
            IpcGeometryClass::unavailable("IPC query failed");
        let empty: IpcGeometryClass<Vec<IpcTrack>> = IpcGeometryClass::available(vec![]);
        assert!(!unavailable.available);
        assert!(unavailable.items.is_empty());
        assert_eq!(unavailable.reason.as_deref(), Some("IPC query failed"));
        assert!(empty.available);
        assert!(empty.items.is_empty());
        assert!(empty.reason.is_none());
    }

    #[test]
    fn inspection_tools_are_registered_in_pcb_routing() {
        let names: Vec<_> = tools().into_iter().map(|t| t.name).collect();
        assert!(names.iter().any(|n| *n == "get_routing_geometry"));
        assert!(names.iter().any(|n| *n == "check_route_clearance"));
        assert!(names.iter().any(|n| *n == "check_via_clearance"));
    }

    #[test]
    fn rotated_rectangle_and_oval_use_exact_shape() {
        let mut p = pad("U1", "1", "GND", 0.0, 0.0);
        p.shape = Some("PssRectangle".into());
        p.size = Some(IpcVector2 { x: 2.0, y: 0.4 });
        p.rotation = 90.0;
        assert_eq!(pad_segment_distance(&p, (0.0, 0.8), (1.0, 0.8)), Some(0.0));
        assert!(pad_segment_distance(&p, (0.8, -1.0), (0.8, 1.0)).unwrap() > 0.5);
        p.shape = Some("PssOval".into());
        p.rotation = 0.0;
        assert_eq!(pad_segment_distance(&p, (-2.0, 0.0), (2.0, 0.0)), Some(0.0));
        assert!((pad_segment_distance(&p, (-2.0, 0.5), (2.0, 0.5)).unwrap() - 0.3).abs() < 1e-9);
    }

    #[test]
    fn rounded_rectangle_and_custom_availability() {
        let mut p = pad("U1", "1", "GND", 0.0, 0.0);
        p.shape = Some("PssRoundrect".into());
        p.size = Some(IpcVector2 { x: 2.0, y: 1.0 });
        p.corner_rounding_ratio = Some(0.25);
        assert_eq!(pad_segment_distance(&p, (-2.0, 0.0), (2.0, 0.0)), Some(0.0));
        p.shape = Some("PssCustom".into());
        p.geometry_supported = false;
        p.geometry_reason = Some("unsupported IPC pad shape: PssCustom".into());
        assert_eq!(pad_segment_distance(&p, (0.0, 0.0), (1.0, 0.0)), None);
    }

    #[test]
    fn line_and_arc_edge_clearance() {
        assert!(
            (segment_distance((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)) - 1.0).abs() < 1e-9
        );
        let d = segment_arc_distance((-0.1, 2.0), (0.1, 2.0), (1.0, 0.0), (0.0, 1.0), (-1.0, 0.0));
        assert!((d - 1.0).abs() < 1e-9);
    }

    #[test]
    fn via_layer_traversal_is_ordered() {
        let geometry = empty_geometry();
        assert_eq!(
            traversed_layers(&geometry.layers.items, "F.Cu", "In2.Cu").unwrap(),
            vec!["F.Cu", "In1.Cu", "In2.Cu"]
        );
        assert_eq!(
            traversed_layers(&geometry.layers.items, "In2.Cu", "In1.Cu").unwrap(),
            vec!["In1.Cu", "In2.Cu"]
        );
        assert_eq!(
            traversed_layers(&geometry.layers.items, "F.Cu", "B.Cu").unwrap(),
            vec!["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
        );
        assert!(traversed_layers(&geometry.layers.items, "F.Cu", "In3.Cu").is_none());
    }

    #[test]
    fn foreign_ground_plane_crossing_supports_conservative_antipad() {
        let mut geometry = empty_geometry();
        geometry.zones.items = vec![IpcPolygonGeometry {
            object_type: "zone_outline".into(),
            reference: None,
            layer: "In1.Cu".into(),
            net_name: Some("GND".into()),
            net_id: Some(1),
            polygons: vec![vec![
                IpcVector2 { x: 0.0, y: 0.0 },
                IpcVector2 { x: 0.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 10.0 },
                IpcVector2 { x: 10.0, y: 0.0 },
            ]],
        }];
        let layers = traversed_layers(&geometry.layers.items, "F.Cu", "In2.Cu").unwrap();
        let (crossings, supported) =
            foreign_zone_crossings(&geometry, &layers, "SIGNAL", (5.0, 5.0), 0.5);
        assert!(supported);
        assert_eq!(crossings.len(), 1);
        assert_eq!(crossings[0]["layer"], "In1.Cu");
        assert_eq!(crossings[0]["zone_net"], "GND");
        assert_eq!(crossings[0]["antipad_required"], true);
        assert_eq!(crossings[0]["required_antipad_radius"], 0.5);
        assert_eq!(crossings[0]["supported"], true);
    }

    #[test]
    fn board_edge_failure_still_fails() {
        let mut geometry = empty_geometry();
        geometry.board_edges.items = vec![IpcBoardEdgeGeometry::Line {
            start: IpcVector2 { x: 0.0, y: 0.0 },
            end: IpcVector2 { x: 10.0, y: 0.0 },
        }];
        let report = calculate_clearance(
            &geometry,
            &[(1.0, 0.1), (2.0, 0.1)],
            "F.Cu",
            "SIGNAL",
            0.2,
            0.2,
            &[],
        );
        assert!(report
            .conflicts
            .iter()
            .any(|conflict| conflict["object_type"] == "board_edge"));
    }
}
