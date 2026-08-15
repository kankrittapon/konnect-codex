//! KiCad 10 IPC API client using NNG + Protocol Buffers.
//!
//! KiCad 10 exposes an IPC API over NNG (nanomsg-next-gen) using protobuf messages.
//! The transport is NNG req/rep over IPC (Unix sockets / Windows named pipes).
//!
//! Socket path: set by KICAD_API_SOCKET env var when KiCAD launches a plugin,
//! or can be manually specified.
//!
//! Protocol: ApiRequest envelope containing a google.protobuf.Any body → ApiResponse.

use crate::gen::kiapi;
use crate::types::*;
use anyhow::{Context, Result};
// NNG SetOpt trait is brought in scope automatically by the nng crate's prelude
use prost::Message;
use std::path::{Path, PathBuf};
use tracing::{debug, warn};

/// Converts KiCAD nanometers to millimeters.
fn nm_to_mm(nm: i64) -> f64 {
    nm as f64 / 1_000_000.0
}

/// Map a BoardLayer enum integer back to a KiCAD layer name string.
fn layer_enum_to_name(layer: i32) -> &'static str {
    match kiapi::board::types::BoardLayer::try_from(layer) {
        Ok(l) => match l {
            kiapi::board::types::BoardLayer::BlFCu => "F.Cu",
            kiapi::board::types::BoardLayer::BlBCu => "B.Cu",
            kiapi::board::types::BoardLayer::BlIn1Cu => "In1.Cu",
            kiapi::board::types::BoardLayer::BlIn2Cu => "In2.Cu",
            kiapi::board::types::BoardLayer::BlFSilkS => "F.SilkS",
            kiapi::board::types::BoardLayer::BlBSilkS => "B.SilkS",
            kiapi::board::types::BoardLayer::BlFMask => "F.Mask",
            kiapi::board::types::BoardLayer::BlBMask => "B.Mask",
            kiapi::board::types::BoardLayer::BlFPaste => "F.Paste",
            kiapi::board::types::BoardLayer::BlBPaste => "B.Paste",
            kiapi::board::types::BoardLayer::BlFCrtYd => "F.CrtYd",
            kiapi::board::types::BoardLayer::BlBCrtYd => "B.CrtYd",
            kiapi::board::types::BoardLayer::BlFFab => "F.Fab",
            kiapi::board::types::BoardLayer::BlBFab => "B.Fab",
            kiapi::board::types::BoardLayer::BlEdgeCuts => "Edge.Cuts",
            _ => "Unknown",
        },
        Err(_) => "Unknown",
    }
}

fn point(p: &kiapi::common::types::Vector2) -> IpcVector2 {
    IpcVector2 {
        x: nm_to_mm(p.x_nm),
        y: nm_to_mm(p.y_nm),
    }
}

fn polyline_points(line: &kiapi::common::types::PolyLine) -> Vec<IpcVector2> {
    line.nodes
        .iter()
        .filter_map(|n| match n.geometry.as_ref()? {
            kiapi::common::types::poly_line_node::Geometry::Point(p) => Some(point(p)),
            // Arc nodes cannot be represented faithfully as a straight polygon here.
            kiapi::common::types::poly_line_node::Geometry::Arc(_) => None,
        })
        .collect()
}

fn polyset_points(set: Option<&kiapi::common::types::PolySet>) -> Vec<Vec<IpcVector2>> {
    set.map(|s| {
        s.polygons
            .iter()
            .filter_map(|p| p.outline.as_ref())
            .map(polyline_points)
            .collect()
    })
    .unwrap_or_default()
}

fn graphic_shape_points(
    shape: Option<&kiapi::common::types::GraphicShape>,
) -> Option<Vec<IpcVector2>> {
    use kiapi::common::types::graphic_shape::Geometry;
    match shape?.geometry.as_ref()? {
        Geometry::Segment(s) => Some(vec![point(s.start.as_ref()?), point(s.end.as_ref()?)]),
        Geometry::Rectangle(r) => {
            let a = r.top_left.as_ref()?;
            let b = r.bottom_right.as_ref()?;
            Some(vec![
                point(a),
                IpcVector2 {
                    x: nm_to_mm(b.x_nm),
                    y: nm_to_mm(a.y_nm),
                },
                point(b),
                IpcVector2 {
                    x: nm_to_mm(a.x_nm),
                    y: nm_to_mm(b.y_nm),
                },
            ])
        }
        Geometry::Polygon(p) => polyset_points(Some(p)).into_iter().next(),
        _ => None,
    }
}

/// Wrap a protobuf message into a prost_types::Any with the correct type_url.
fn pack_any<M: Message>(msg: &M, type_name: &str) -> prost_types::Any {
    let mut buf = Vec::new();
    msg.encode(&mut buf).expect("protobuf encode failed");
    prost_types::Any {
        type_url: format!("type.googleapis.com/{}", type_name),
        value: buf,
    }
}

/// Decode a prost_types::Any into a specific protobuf message type.
fn unpack_any<M: Message + Default>(any: &prost_types::Any) -> Result<M> {
    M::decode(any.value.as_slice()).context("Failed to decode protobuf Any body")
}

fn unpack_required<M: Message + Default>(
    response: Option<prost_types::Any>,
    command_name: &str,
) -> Result<M> {
    let response =
        response.with_context(|| format!("KiCad returned no {command_name} response payload"))?;
    unpack_any(&response)
}

fn ensure_item_request_ok(status: i32, operation: &str) -> Result<()> {
    let status = kiapi::common::types::ItemRequestStatus::try_from(status)
        .unwrap_or(kiapi::common::types::ItemRequestStatus::IrsUnknown);
    if status != kiapi::common::types::ItemRequestStatus::IrsOk {
        anyhow::bail!(
            "KiCad {operation} request failed with {}",
            status.as_str_name()
        );
    }
    Ok(())
}

/// Marker error carried (via anyhow's error chain) by every failure where no
/// request completed a round-trip with a live KiCad: no socket path
/// configured, or the NNG dial/send failed.
///
/// Callers must classify with [`IpcFailure::from_error`], never by matching
/// error text.
#[derive(Debug)]
pub struct TransportUnreachable;

impl std::fmt::Display for TransportUnreachable {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "KiCad IPC transport unreachable")
    }
}

impl std::error::Error for TransportUnreachable {}

/// Why an IPC operation failed, for callers deciding whether a file-based
/// fallback is safe.
///
/// `Unreachable` means the transport never delivered the request — IPC is
/// unconfigured (empty socket path) or the dial/send failed — so no live
/// KiCad can be holding the board, and editing the board file directly
/// cannot race an editor.
///
/// `Rejected` is everything else, including any error after a request was
/// delivered (a receive timeout may mean KiCad is still processing it).
/// KiCad is — or may be — alive on the other end, so a file edit could be
/// silently overwritten on its next save. Fail closed.
#[derive(Debug)]
pub enum IpcFailure {
    Unreachable(String),
    Rejected(String),
}

impl IpcFailure {
    /// Classify an error from any [`KiCadIpcClient`] operation by walking its
    /// chain for the [`TransportUnreachable`] marker — never by matching
    /// message text.
    pub fn from_error(error: anyhow::Error) -> Self {
        let message = format!("{error:#}");
        if error
            .chain()
            .any(|cause| cause.is::<TransportUnreachable>())
        {
            IpcFailure::Unreachable(message)
        } else {
            IpcFailure::Rejected(message)
        }
    }

    pub fn message(&self) -> &str {
        match self {
            IpcFailure::Unreachable(message) | IpcFailure::Rejected(message) => message,
        }
    }
}

impl std::fmt::Display for IpcFailure {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}", self.message())
    }
}

pub struct KiCadIpcClient {
    socket_path: String,
    kicad_token: String,
    client_name: String,
}

impl KiCadIpcClient {
    /// Create a client connecting to the given IPC socket path.
    /// If empty, tries KICAD_API_SOCKET environment variable.
    pub fn new(socket_path: impl Into<String>) -> Self {
        let path = socket_path.into();
        let effective_path = if path.is_empty() {
            std::env::var("KICAD_API_SOCKET").unwrap_or_default()
        } else {
            path
        };
        KiCadIpcClient {
            socket_path: effective_path,
            kicad_token: std::env::var("KICAD_API_TOKEN").unwrap_or_default(),
            client_name: format!("konnect-{}", std::process::id()),
        }
    }

    /// Create a client with an explicit API token.
    ///
    /// KiCad supplies this token to executable plugins through
    /// `KICAD_API_TOKEN`. This constructor is useful for clients that obtain
    /// the connection details through another discovery mechanism.
    pub fn new_with_token(socket_path: impl Into<String>, token: impl Into<String>) -> Self {
        let mut client = Self::new(socket_path);
        client.kicad_token = token.into();
        client
    }

    /// Send a protobuf command and return the response Any.
    fn send_command(
        &self,
        command: &impl Message,
        type_name: &str,
    ) -> Result<Option<prost_types::Any>> {
        if self.socket_path.is_empty() {
            return Err(anyhow::Error::new(TransportUnreachable).context(
                "KiCAD IPC socket path not configured. To fix: \
                 (1) in KiCAD, enable Edit > Preferences > Plugins > 'Enable KiCad API' \
                 and copy the listed ipc:// address; \
                 (2) paste it into the 'IPC Socket' field of the Konnect settings dialog \
                 (Tools > External Plugins > Konnect) and save; \
                 (3) restart the AI client so the server rereads settings. \
                 Alternatively set ipc_socket_path in konnect-settings.json or launch \
                 via KiCAD (which sets KICAD_API_SOCKET). \
                 Full guide: https://github.com/mixelpixx/Konnect/blob/main/docs/TROUBLESHOOTING.md",
            ));
        }

        let request = kiapi::common::ApiRequest {
            header: Some(kiapi::common::ApiRequestHeader {
                kicad_token: self.kicad_token.clone(),
                client_name: self.client_name.clone(),
            }),
            message: Some(pack_any(command, type_name)),
        };

        let request_bytes = request.encode_to_vec();
        debug!(
            "[BETA] IPC → {} ({} bytes) to {}",
            type_name,
            request_bytes.len(),
            self.socket_path
        );

        // Connect via NNG req0 socket
        let socket =
            nng::Socket::new(nng::Protocol::Req0).context("Failed to create NNG socket")?;

        // Bound every step: a busy or wedged KiCAD must produce an error the
        // tools can surface, never an indefinite hang (the predecessor
        // project's sync/autoroute hangs blocked for >600 s on exactly this).
        // 30 s receive allows slow board operations like zone refills.
        use nng::options::Options;
        socket
            .set_opt::<nng::options::SendTimeout>(Some(std::time::Duration::from_secs(5)))
            .context("Failed to set NNG send timeout")?;
        socket
            .set_opt::<nng::options::RecvTimeout>(Some(std::time::Duration::from_secs(30)))
            .context("Failed to set NNG receive timeout")?;

        // Build the dial URL. inproc:// is same-process only — used by the
        // mock-KiCAD test servers, where it avoids TCP port races entirely.
        let dial_url = if self.socket_path.starts_with("ipc://")
            || self.socket_path.starts_with("tcp://")
            || self.socket_path.starts_with("inproc://")
        {
            self.socket_path.clone()
        } else {
            format!("ipc://{}", self.socket_path)
        };

        socket.dial(&dial_url).map_err(|error| {
            anyhow::Error::new(TransportUnreachable).context(format!(
                "Cannot connect to KiCAD IPC at {dial_url}: {error}"
            ))
        })?;

        // Send request
        let msg = nng::Message::from(request_bytes.as_slice());
        socket.send(msg).map_err(|(_, error)| {
            anyhow::Error::new(TransportUnreachable).context(format!("NNG send failed: {error}"))
        })?;

        // Receive response
        let reply = socket
            .recv()
            .map_err(|e| anyhow::anyhow!("NNG recv failed: {}", e))?;

        let response = kiapi::common::ApiResponse::decode(reply.as_slice())
            .context("Failed to decode ApiResponse")?;

        // A response without an envelope status is malformed, not success.
        let status = response
            .status
            .as_ref()
            .context("KiCad IPC response is missing its status")?;
        let code = status.status();
        if code != kiapi::common::ApiStatusCode::AsOk {
            let msg = if status.error_message.is_empty() {
                format!("{:?}", code)
            } else {
                status.error_message.clone()
            };
            debug!("[BETA] IPC ← error: {} ({})", msg, code.as_str_name());
            anyhow::bail!("KiCad IPC error: {} ({})", msg, code.as_str_name());
        }

        debug!("[BETA] IPC ← OK");
        Ok(response.message)
    }

    // ─── Public API (same interface as before, tools don't change) ───────

    /// Check if KiCAD is reachable.
    pub fn ping(&self) -> Result<bool> {
        let ping = kiapi::common::commands::Ping {};
        match self.send_command(&ping, "kiapi.common.commands.Ping") {
            Ok(_) => Ok(true),
            Err(e) => {
                warn!("[BETA] Ping failed: {}", e);
                Ok(false)
            }
        }
    }

    /// Get the list of open documents (boards).
    pub fn get_open_documents(&self) -> Result<Vec<kiapi::common::types::DocumentSpecifier>> {
        let cmd = kiapi::common::commands::GetOpenDocuments {
            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
        };
        let response_any = self.send_command(&cmd, "kiapi.common.commands.GetOpenDocuments")?;
        if let Some(any) = response_any {
            let resp: kiapi::common::commands::GetOpenDocumentsResponse = unpack_any(&any)?;
            Ok(resp.documents)
        } else {
            Ok(vec![])
        }
    }

    /// Get the first open PCB's DocumentSpecifier (needed for most commands).
    fn get_board_document(&self) -> Result<kiapi::common::types::DocumentSpecifier> {
        let docs = self.get_open_documents()?;
        docs.into_iter().next().ok_or_else(|| {
            anyhow::anyhow!("No PCB document is open in KiCAD. Open a board file first.")
        })
    }

    /// Find the open document matching `requested`, so a path-bearing MCP
    /// request operates on the board it names — not whichever board happens
    /// to be first in KiCad's open-document list. Live verification caught
    /// exactly this: with the user's own project focused and the target
    /// board open behind it, first-document targeting either fails or, worse,
    /// would mutate the wrong board.
    pub fn find_open_board(
        &self,
        requested: &Path,
    ) -> Result<kiapi::common::types::DocumentSpecifier> {
        let docs = self.get_open_documents()?;
        if docs.is_empty() {
            anyhow::bail!("No PCB document is open in KiCAD. Open a board file first.");
        }
        let mut open_names = Vec::new();
        for doc in docs {
            if let Some(path) = board_document_path(&doc) {
                if paths_refer_to_same_board(requested, &path) {
                    return Ok(doc);
                }
                open_names.push(path.display().to_string());
            }
        }
        anyhow::bail!(
            "requested board '{}' is not open in KiCAD (open boards: {})",
            requested.display(),
            open_names.join(", ")
        )
    }

    /// Fail closed unless the requested board is open in the IPC session.
    pub fn ensure_board_is_active(&self, requested: &Path) -> Result<()> {
        self.find_open_board(requested).map(|_| ())
    }

    fn make_header(&self) -> Result<kiapi::common::types::ItemHeader> {
        Ok(header_for(self.get_board_document()?))
    }

    /// Get all nets on the board.
    pub fn get_nets(&self) -> Result<Vec<IpcNet>> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::GetNets {
            board: Some(doc),
            netclass_filter: vec![],
        };
        let response_any = self.send_command(&cmd, "kiapi.board.commands.GetNets")?;
        if let Some(any) = response_any {
            let resp: kiapi::board::commands::NetsResponse = unpack_any(&any)?;
            Ok(resp
                .nets
                .iter()
                .map(|n| IpcNet {
                    name: n.name.clone(),
                    netcode: n.code.as_ref().map(|c| c.value).unwrap_or(0),
                })
                .collect())
        } else {
            Ok(vec![])
        }
    }

    /// Get board items by type.
    pub fn get_items(
        &self,
        item_type: kiapi::common::types::KiCadObjectType,
    ) -> Result<Vec<prost_types::Any>> {
        self.get_items_in(self.get_board_document()?, item_type)
    }

    /// As [`Self::get_items`], targeting a specific open document.
    pub fn get_items_in(
        &self,
        document: kiapi::common::types::DocumentSpecifier,
        item_type: kiapi::common::types::KiCadObjectType,
    ) -> Result<Vec<prost_types::Any>> {
        let header = header_for(document);
        let cmd = kiapi::common::commands::GetItems {
            header: Some(header),
            types: vec![item_type as i32],
        };
        let response_any = self.send_command(&cmd, "kiapi.common.commands.GetItems")?;
        if let Some(any) = response_any {
            let resp: kiapi::common::commands::GetItemsResponse = unpack_any(&any)?;
            Ok(resp.items)
        } else {
            Ok(vec![])
        }
    }

    /// Resolve DRC-reported KIID values to board identity using read-only IPC
    /// item queries. Unknown IDs are omitted rather than inferred.
    pub fn resolve_board_item_ids(&self, ids: &[String]) -> Result<Vec<IpcBoardItemIdentity>> {
        use kiapi::common::types::KiCadObjectType as T;
        use std::collections::HashSet;

        let wanted: HashSet<&str> = ids.iter().map(String::as_str).collect();
        let mut resolved = Vec::new();
        let id_value = |id: &Option<kiapi::common::types::Kiid>| {
            id.as_ref().map(|id| id.value.clone()).unwrap_or_default()
        };
        let position = |p: Option<&kiapi::common::types::Vector2>| {
            p.map(|p| IpcVector2 {
                x: nm_to_mm(p.x_nm),
                y: nm_to_mm(p.y_nm),
            })
        };

        for item in self.get_items(T::KotPcbTrace)? {
            let track = kiapi::board::types::Track::decode(item.value.as_slice())?;
            let kiid = id_value(&track.id);
            if wanted.contains(kiid.as_str()) {
                resolved.push(IpcBoardItemIdentity {
                    kiid,
                    item_type: "track".into(),
                    reference: None,
                    pad: None,
                    net: track.net.map(|n| n.name),
                    layer: Some(layer_enum_to_name(track.layer).into()),
                    position: None,
                });
            }
        }
        for item in self.get_items(T::KotPcbArc)? {
            let arc = kiapi::board::types::Arc::decode(item.value.as_slice())?;
            let kiid = id_value(&arc.id);
            if wanted.contains(kiid.as_str()) {
                resolved.push(IpcBoardItemIdentity {
                    kiid,
                    item_type: "track_arc".into(),
                    reference: None,
                    pad: None,
                    net: arc.net.map(|n| n.name),
                    layer: Some(layer_enum_to_name(arc.layer).into()),
                    position: None,
                });
            }
        }
        for item in self.get_items(T::KotPcbVia)? {
            let via = kiapi::board::types::Via::decode(item.value.as_slice())?;
            let kiid = id_value(&via.id);
            if wanted.contains(kiid.as_str()) {
                resolved.push(IpcBoardItemIdentity {
                    kiid,
                    item_type: "via".into(),
                    reference: None,
                    pad: None,
                    net: via.net.map(|n| n.name),
                    layer: None,
                    position: position(via.position.as_ref()),
                });
            }
        }
        for item in self.get_items(T::KotPcbFootprint)? {
            let fp = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())?;
            let reference = fp
                .reference_field
                .as_ref()
                .and_then(|f| f.text.as_ref())
                .and_then(|t| t.text.as_ref())
                .map(|t| t.text.clone())
                .filter(|s| !s.is_empty());
            let fp_kiid = id_value(&fp.id);
            if wanted.contains(fp_kiid.as_str()) {
                resolved.push(IpcBoardItemIdentity {
                    kiid: fp_kiid,
                    item_type: "footprint".into(),
                    reference: reference.clone(),
                    pad: None,
                    net: None,
                    layer: Some(layer_enum_to_name(fp.layer).into()),
                    position: position(fp.position.as_ref()),
                });
            }
            for child in fp
                .definition
                .as_ref()
                .map(|d| d.items.as_slice())
                .unwrap_or_default()
            {
                if !child.type_url.ends_with(".Pad") {
                    continue;
                }
                let pad = kiapi::board::types::Pad::decode(child.value.as_slice())?;
                let pad_kiid = id_value(&pad.id);
                if wanted.contains(pad_kiid.as_str()) {
                    let layer = pad
                        .pad_stack
                        .as_ref()
                        .and_then(|stack| stack.layers.first())
                        .map(|layer| layer_enum_to_name(*layer).to_owned());
                    resolved.push(IpcBoardItemIdentity {
                        kiid: pad_kiid,
                        item_type: "pad".into(),
                        reference: reference.clone(),
                        pad: Some(pad.number),
                        net: pad.net.map(|n| n.name),
                        layer,
                        position: position(pad.position.as_ref()),
                    });
                }
            }
        }

        Ok(resolved)
    }

    /// List all footprints on the board.
    pub fn list_footprints(&self) -> Result<Vec<IpcFootprint>> {
        self.list_footprints_in(self.get_board_document()?)
    }

    /// As [`Self::list_footprints`], targeting a specific open document.
    pub fn list_footprints_in(
        &self,
        document: kiapi::common::types::DocumentSpecifier,
    ) -> Result<Vec<IpcFootprint>> {
        let items = self.get_items_in(
            document,
            kiapi::common::types::KiCadObjectType::KotPcbFootprint,
        )?;
        let mut footprints = Vec::new();
        for item in &items {
            if let Ok(fp) = kiapi::board::types::FootprintInstance::decode(item.value.as_slice()) {
                let pos = fp.position.as_ref();
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.clone())
                    .unwrap_or_default();
                let val_text = fp
                    .value_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.clone())
                    .unwrap_or_default();
                let lib_id = fp
                    .definition
                    .as_ref()
                    .and_then(|d| d.id.as_ref())
                    .map(|id| format!("{}:{}", id.library_nickname, id.entry_name))
                    .unwrap_or_default();
                footprints.push(IpcFootprint {
                    reference: ref_text,
                    value: val_text,
                    footprint: lib_id,
                    position: IpcVector2 {
                        x: pos.map(|p| nm_to_mm(p.x_nm)).unwrap_or(0.0),
                        y: pos.map(|p| nm_to_mm(p.y_nm)).unwrap_or(0.0),
                    },
                    rotation: fp
                        .orientation
                        .as_ref()
                        .map(|a| a.value_degrees)
                        .unwrap_or(0.0),
                    layer: layer_enum_to_name(fp.layer).to_string(),
                });
            }
        }
        Ok(footprints)
    }

    /// Create items on the board.
    ///
    /// `created_items` is documented as the status of each item *to be*
    /// created: a rejected item still occupies a result slot (with e.g.
    /// `ISC_INVALID_DATA` and no item attached), so counting the vector's
    /// length would report phantom successes. Only results whose status is
    /// `ISC_OK` — or whose status was left at the protobuf default while an
    /// item came back, since proto3 cannot distinguish "unset" from an
    /// explicit `ISC_UNKNOWN` — count as created; anything else fails with
    /// KiCad's own per-item reasons. (Outcome semantics follow emolitor's
    /// PR #66.)
    pub fn create_items(&self, items: Vec<prost_types::Any>) -> Result<()> {
        self.create_items_in(self.get_board_document()?, items)
    }

    /// As [`Self::create_items`], targeting a specific open document.
    pub fn create_items_in(
        &self,
        document: kiapi::common::types::DocumentSpecifier,
        items: Vec<prost_types::Any>,
    ) -> Result<()> {
        if items.is_empty() {
            return Ok(());
        }
        let expected_count = items.len();
        let header = header_for(document);
        let cmd = kiapi::common::commands::CreateItems {
            header: Some(header),
            items,
            container: None,
        };
        let response = unpack_required::<kiapi::common::commands::CreateItemsResponse>(
            self.send_command(&cmd, "kiapi.common.commands.CreateItems")?,
            "CreateItems",
        )?;
        ensure_item_request_ok(response.status, "item creation")?;
        if response.created_items.is_empty() {
            // KiCad 10.0's observed behaviour for a request it ignores: an
            // IRS_OK response carrying no per-item results at all.
            anyhow::bail!(
                "KiCad created no items: the CreateItems response carried no items at all \
                 ({expected_count} requested)"
            );
        }
        let mut created = 0usize;
        let mut rejections = Vec::new();
        for (index, result) in response.created_items.iter().enumerate() {
            use kiapi::common::commands::ItemStatusCode;
            let code = result
                .status
                .as_ref()
                .map(|status| status.code())
                .unwrap_or(ItemStatusCode::IscUnknown);
            let is_created = match code {
                ItemStatusCode::IscOk => true,
                // A defaulted status is only evidence of success when the
                // created item itself came back alongside it.
                ItemStatusCode::IscUnknown => result.item.is_some(),
                _ => false,
            };
            if is_created {
                created += 1;
            } else {
                let message = result
                    .status
                    .as_ref()
                    .map(|status| status.error_message.as_str())
                    .filter(|message| !message.is_empty())
                    .unwrap_or("no error message");
                rejections.push(format!("item {index}: {} ({message})", code.as_str_name()));
            }
        }
        if created != expected_count {
            anyhow::bail!(
                "KiCad created {created} of {expected_count} requested items{}{}",
                if rejections.is_empty() { "" } else { ": " },
                rejections.join("; ")
            );
        }
        Ok(())
    }

    /// Update existing items by KIID. Generic wrapper mirroring create_items/delete_items;
    /// each `Any` must be a fully-formed board item with an existing `id` populated.
    pub fn update_items(&self, items: Vec<prost_types::Any>) -> Result<()> {
        if items.is_empty() {
            return Ok(());
        }
        let expected_count = items.len();
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::UpdateItems {
            header: Some(header),
            items,
        };
        let response = unpack_required::<kiapi::common::commands::UpdateItemsResponse>(
            self.send_command(&cmd, "kiapi.common.commands.UpdateItems")?,
            "UpdateItems",
        )?;
        ensure_item_request_ok(response.status, "item update")?;
        if response.updated_items.len() != expected_count {
            anyhow::bail!(
                "KiCad returned {} update results for {} requested items",
                response.updated_items.len(),
                expected_count
            );
        }
        for result in response.updated_items {
            let Some(status) = result.status else {
                anyhow::bail!("KiCad returned an update result without item status");
            };
            if status.code() != kiapi::common::commands::ItemStatusCode::IscOk {
                anyhow::bail!(
                    "KiCad item update failed: {} ({})",
                    status.error_message,
                    status.code().as_str_name()
                );
            }
        }
        Ok(())
    }

    /// Delete items by KIID.
    pub fn delete_items(&self, ids: Vec<String>) -> Result<()> {
        if ids.is_empty() {
            return Ok(());
        }
        let expected_count = ids.len();
        let mut expected_ids = ids
            .iter()
            .cloned()
            .collect::<std::collections::HashSet<_>>();
        if expected_ids.len() != expected_count {
            anyhow::bail!("delete request contains duplicate item identifiers");
        }
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::DeleteItems {
            header: Some(header),
            item_ids: ids
                .iter()
                .map(|id| kiapi::common::types::Kiid { value: id.clone() })
                .collect(),
        };
        let response = unpack_required::<kiapi::common::commands::DeleteItemsResponse>(
            self.send_command(&cmd, "kiapi.common.commands.DeleteItems")?,
            "DeleteItems",
        )?;
        ensure_item_request_ok(response.status, "item deletion")?;
        // KiCad 10 builds per-item results in
        // API_HANDLER_EDITOR::handleDeleteItems and then never attaches them
        // to the response, so `deleted_items` comes back empty on every
        // successful delete. Treating that as failure made delete_component
        // report "0 deletion results" for deletions that had in fact happened
        // (#116). An empty list carries no information either way, so callers
        // that need certainty verify the item is gone; a NON-empty list is
        // still validated strictly, so this stays correct if KiCad starts
        // populating it.
        if response.deleted_items.is_empty() {
            return Ok(());
        }
        if response.deleted_items.len() != expected_count {
            anyhow::bail!(
                "KiCad returned {} deletion results for {} requested items",
                response.deleted_items.len(),
                expected_count
            );
        }
        for result in response.deleted_items {
            let status = result.status();
            let id = result
                .id
                .context("KiCad returned a deletion result without an item identifier")?
                .value;
            if !expected_ids.remove(&id) {
                anyhow::bail!("KiCad returned a deletion result for unexpected item '{id}'");
            }
            if status != kiapi::common::commands::ItemDeletionStatus::IdsOk {
                anyhow::bail!(
                    "KiCad failed to delete item '{}': {}",
                    id,
                    status.as_str_name()
                );
            }
        }
        Ok(())
    }

    /// Refill zones on the board.
    pub fn refill_zones(&self) -> Result<()> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::RefillZones {
            board: Some(doc),
            zones: vec![],
        };
        self.send_command(&cmd, "kiapi.board.commands.RefillZones")?;
        Ok(())
    }

    /// Save the open board document.
    pub fn save_board(&self) -> Result<()> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::common::commands::SaveDocument {
            document: Some(doc),
        };
        self.send_command(&cmd, "kiapi.common.commands.SaveDocument")?;
        Ok(())
    }

    /// Begin a commit (undo group).
    pub fn begin_commit(&self) -> Result<String> {
        let cmd = kiapi::common::commands::BeginCommit {};
        let response_any = self.send_command(&cmd, "kiapi.common.commands.BeginCommit")?;
        let response: kiapi::common::commands::BeginCommitResponse =
            unpack_required(response_any, "BeginCommit")?;
        let id = response
            .id
            .context("KiCad returned BeginCommit without a commit identifier")?
            .value;
        if id.is_empty() {
            anyhow::bail!("KiCad returned an empty commit identifier");
        }
        Ok(id)
    }

    /// End a commit (push or drop).
    pub fn end_commit(
        &self,
        commit_id: &str,
        action: kiapi::common::commands::CommitAction,
        message: &str,
    ) -> Result<()> {
        let cmd = kiapi::common::commands::EndCommit {
            id: Some(kiapi::common::types::Kiid {
                value: commit_id.to_string(),
            }),
            action: action as i32,
            message: message.to_string(),
        };
        let _: kiapi::common::commands::EndCommitResponse = unpack_required(
            self.send_command(&cmd, "kiapi.common.commands.EndCommit")?,
            "EndCommit",
        )?;
        Ok(())
    }

    /// Push (commit) changes.
    pub fn push_commit(&self, commit_id: &str, description: &str) -> Result<()> {
        self.end_commit(
            commit_id,
            kiapi::common::commands::CommitAction::CmaCommit,
            description,
        )
    }

    /// Drop (rollback) changes.
    pub fn drop_commit(&self, commit_id: &str) -> Result<()> {
        self.end_commit(
            commit_id,
            kiapi::common::commands::CommitAction::CmaDrop,
            "",
        )
    }

    /// Run a multi-step mutation as one KiCad undo transaction.
    ///
    /// Any operation error, or a failure to publish the commit, triggers a
    /// best-effort drop so callers never knowingly leave a partial batch.
    pub fn run_commit<T>(
        &self,
        description: &str,
        operation: impl FnOnce(&Self) -> Result<T>,
    ) -> Result<T> {
        let commit_id = self.begin_commit()?;
        let operation_result =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| operation(self)));
        match operation_result {
            Err(panic) => {
                if let Err(rollback_error) = self.drop_commit(&commit_id) {
                    anyhow::bail!("KiCad batch panicked and rollback failed ({rollback_error})");
                }
                std::panic::resume_unwind(panic)
            }
            Ok(Ok(value)) => {
                if let Err(commit_error) = self.push_commit(&commit_id, description) {
                    let rollback_error = self.drop_commit(&commit_id).err();
                    if let Some(rollback_error) = rollback_error {
                        anyhow::bail!(
                            "failed to publish KiCad commit ({commit_error}); rollback also failed ({rollback_error})"
                        );
                    }
                    return Err(commit_error)
                        .context("failed to publish KiCad commit; changes dropped");
                }
                Ok(value)
            }
            Ok(Err(operation_error)) => {
                if let Err(rollback_error) = self.drop_commit(&commit_id) {
                    anyhow::bail!(
                        "KiCad batch failed ({operation_error}); rollback also failed ({rollback_error})"
                    );
                }
                Err(operation_error).context("KiCad batch failed; changes dropped")
            }
        }
    }

    // ─── PCB Item Operations (real protobuf implementations) ───────────

    /// Resolve a net name to its net code by querying GetNets.
    pub fn resolve_net_code(&self, net_name: &str) -> Result<i32> {
        let nets = self.get_nets()?;
        nets.iter()
            .find(|n| n.name == net_name)
            .map(|n| n.netcode)
            .ok_or_else(|| anyhow::anyhow!("Net '{}' not found on board", net_name))
    }

    /// Find a footprint by reference and return its IpcFootprint + KIID.
    pub fn get_footprint(&self, reference: &str) -> Result<Option<IpcFootprint>> {
        let footprints = self.list_footprints()?;
        Ok(footprints.into_iter().find(|fp| fp.reference == reference))
    }

    /// Find a footprint's KIID by reference.
    fn find_footprint_kiid(&self, reference: &str) -> Result<String> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(fp) = kiapi::board::types::FootprintInstance::decode(item.value.as_slice()) {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    if let Some(id) = &fp.id {
                        return Ok(id.value.clone());
                    }
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found on board", reference)
    }

    /// Add a track segment to the board.
    #[allow(clippy::too_many_arguments)]
    pub fn add_track(
        &self,
        net_name: &str,
        layer: &str,
        width: f64,
        x1: f64,
        y1: f64,
        x2: f64,
        y2: f64,
    ) -> Result<()> {
        let net_code = self.resolve_net_code(net_name)?;
        let track = crate::builders::build_track(net_name, net_code, layer, width, x1, y1, x2, y2);
        let any = crate::builders::pack_any(&track, "kiapi.board.types.Track");
        self.create_items(vec![any])?;
        Ok(())
    }

    /// Add a through via (F.Cu → B.Cu) to the board.
    ///
    /// Uses the protobuf `Via` message via `create_items`, the same path as
    /// [`add_track`](Self::add_track). The previous implementation sent a bare
    /// `(via …)` S-expression through `ParseAndCreateItemsFromString`, which
    /// returned success but never created the via (see `builders::build_via`).
    pub fn add_via(&self, net_name: &str, x: f64, y: f64, drill: f64, pad_size: f64) -> Result<()> {
        let net_code = self.resolve_net_code(net_name)?;
        let via = crate::builders::build_via(net_name, net_code, x, y, drill, pad_size);
        let any = crate::builders::pack_any(&via, "kiapi.board.types.Via");
        self.create_items(vec![any])?;
        Ok(())
    }

    /// Delete a track by UUID.
    pub fn delete_track(&self, uuid: &str) -> Result<()> {
        self.ensure_item_kind(
            uuid,
            kiapi::common::types::KiCadObjectType::KotPcbTrace,
            "trace",
        )?;
        self.delete_items(vec![uuid.to_string()])
    }

    /// Delete a via by its KiCad-native UUID.
    pub fn delete_via(&self, uuid: &str) -> Result<()> {
        self.ensure_item_kind(
            uuid,
            kiapi::common::types::KiCadObjectType::KotPcbVia,
            "via",
        )?;
        self.delete_items(vec![uuid.to_string()])
    }

    fn ensure_item_kind(
        &self,
        uuid: &str,
        expected: kiapi::common::types::KiCadObjectType,
        kind: &str,
    ) -> Result<()> {
        let items = self.get_items(expected)?;
        let found = match expected {
            kiapi::common::types::KiCadObjectType::KotPcbTrace => items.iter().any(|item| {
                kiapi::board::types::Track::decode(item.value.as_slice())
                    .ok()
                    .and_then(|v| v.id.map(|id| id.value == uuid))
                    .unwrap_or(false)
            }),
            kiapi::common::types::KiCadObjectType::KotPcbVia => items.iter().any(|item| {
                kiapi::board::types::Via::decode(item.value.as_slice())
                    .ok()
                    .and_then(|v| v.id.map(|id| id.value == uuid))
                    .unwrap_or(false)
            }),
            _ => false,
        };
        if found {
            return Ok(());
        }
        anyhow::bail!("{kind} UUID '{uuid}' was not found on the active board")
    }

    /// Query tracks, optionally filtered by net and/or layer.
    pub fn get_tracks(
        &self,
        net_filter: Option<&str>,
        layer_filter: Option<&str>,
    ) -> Result<Vec<IpcTrack>> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbTrace)?;
        let mut tracks = Vec::new();
        for item in &items {
            if let Ok(track) = kiapi::board::types::Track::decode(item.value.as_slice()) {
                let net_name = track.net.as_ref().map(|n| n.name.as_str()).unwrap_or("");
                let layer_name = layer_enum_to_name(track.layer);

                // Apply net filter
                if let Some(nf) = net_filter {
                    if net_name != nf {
                        continue;
                    }
                }
                // Apply layer filter
                if let Some(lf) = layer_filter {
                    if layer_name != lf {
                        continue;
                    }
                }

                let start = track.start.as_ref();
                let end = track.end.as_ref();
                tracks.push(IpcTrack {
                    kiid: track
                        .id
                        .as_ref()
                        .map(|id| id.value.clone())
                        .unwrap_or_default(),
                    net_name: net_name.to_string(),
                    layer: layer_name.to_string(),
                    width: track
                        .width
                        .as_ref()
                        .map(|w| crate::builders::nm_to_mm(w.value_nm))
                        .unwrap_or(0.25),
                    start: IpcVector2 {
                        x: start
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: start
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                    end: IpcVector2 {
                        x: end
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: end
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                });
            }
        }
        Ok(tracks)
    }

    /// Query vias, optionally filtered by net name.
    pub fn get_vias(&self, net_filter: Option<&str>) -> Result<Vec<IpcViaGeometry>> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbVia)?;
        let mut vias = Vec::new();
        for item in &items {
            if let Ok(via) = kiapi::board::types::Via::decode(item.value.as_slice()) {
                let net_name = via.net.as_ref().map(|n| n.name.as_str()).unwrap_or("");
                if let Some(nf) = net_filter {
                    if net_name != nf {
                        continue;
                    }
                }
                let pos = via.position.as_ref();
                let stack = via.pad_stack.as_ref();
                let copper = stack
                    .and_then(|s| s.copper_layers.first())
                    .and_then(|l| l.size.as_ref());
                let drill = stack
                    .and_then(|s| s.drill.as_ref())
                    .and_then(|d| d.diameter.as_ref());
                vias.push(IpcViaGeometry {
                    kiid: via
                        .id
                        .as_ref()
                        .map(|id| id.value.clone())
                        .unwrap_or_default(),
                    position: IpcVector2 {
                        x: pos
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: pos
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                    size: copper.map(|s| IpcVector2 {
                        x: crate::builders::nm_to_mm(s.x_nm),
                        y: crate::builders::nm_to_mm(s.y_nm),
                    }),
                    drill: drill.map(|s| IpcVector2 {
                        x: crate::builders::nm_to_mm(s.x_nm),
                        y: crate::builders::nm_to_mm(s.y_nm),
                    }),
                    layers: stack
                        .map(|s| {
                            s.layers
                                .iter()
                                .map(|l| layer_enum_to_name(*l).to_string())
                                .collect()
                        })
                        .unwrap_or_default(),
                    net_name: net_name.to_string(),
                    net_id: via
                        .net
                        .as_ref()
                        .and_then(|n| n.code.as_ref())
                        .map(|c| c.value)
                        .unwrap_or(0),
                });
            }
        }
        Ok(vias)
    }

    /// Convert a native `Zone` into an [`IpcRuleArea`], returning `None` if it is
    /// not a rule area (e.g. an ordinary copper-pour zone) or fails the layer filter.
    fn zone_to_rule_area(
        zone: &kiapi::board::types::Zone,
        layer_filter: Option<&str>,
        owner: &str,
        footprint_kiid: Option<String>,
        footprint_reference: Option<String>,
    ) -> Option<IpcRuleArea> {
        let kiapi::board::types::zone::Settings::RuleAreaSettings(settings) =
            zone.settings.as_ref()?
        else {
            return None;
        };
        let layers: Vec<String> = zone
            .layers
            .iter()
            .map(|l| layer_enum_to_name(*l).to_string())
            .collect();
        if let Some(lf) = layer_filter {
            if !layers.iter().any(|l| l == lf) {
                return None;
            }
        }
        Some(IpcRuleArea {
            kiid: zone
                .id
                .as_ref()
                .map(|id| id.value.clone())
                .unwrap_or_default(),
            name: zone.name.clone(),
            layers,
            bounds: polyset_points(zone.outline.as_ref()),
            keepout_copper: settings.keepout_copper,
            keepout_vias: settings.keepout_vias,
            keepout_tracks: settings.keepout_tracks,
            keepout_pads: settings.keepout_pads,
            keepout_footprints: settings.keepout_footprints,
            owner: owner.to_string(),
            footprint_kiid,
            footprint_reference,
        })
    }

    /// Query rule areas / keepouts, optionally filtered by layer.
    ///
    /// Distinct from copper-pour zones: only zones with `RuleAreaSettings` are
    /// returned. Covers both board-level rule areas (top-level `KotPcbZone`
    /// items) and rule areas embedded inside a footprint's own definition
    /// (`FootprintInstance.definition.items`), which are NOT surfaced by
    /// `get_items(KotPcbZone)` alone.
    pub fn get_rule_areas(&self, layer_filter: Option<&str>) -> Result<Vec<IpcRuleArea>> {
        use kiapi::common::types::KiCadObjectType as T;

        let mut rule_areas = Vec::new();

        // Board-owned rule areas.
        for item in self.get_items(T::KotPcbZone)? {
            let Ok(zone) = kiapi::board::types::Zone::decode(item.value.as_slice()) else {
                continue;
            };
            if let Some(ra) = Self::zone_to_rule_area(&zone, layer_filter, "board", None, None) {
                rule_areas.push(ra);
            }
        }

        // Footprint-owned rule areas embedded in each footprint's definition.
        for item in self.get_items(T::KotPcbFootprint)? {
            let Ok(fp) = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            else {
                continue;
            };
            let fp_kiid = fp.id.as_ref().map(|id| id.value.clone());
            let fp_reference = fp
                .reference_field
                .as_ref()
                .and_then(|f| f.text.as_ref())
                .and_then(|t| t.text.as_ref())
                .map(|t| t.text.clone())
                .filter(|s| !s.is_empty());
            let Some(children) = fp.definition.as_ref().map(|d| d.items.as_slice()) else {
                continue;
            };
            for child in children {
                if !child.type_url.ends_with(".Zone") {
                    continue;
                }
                let Ok(zone) = kiapi::board::types::Zone::decode(child.value.as_slice()) else {
                    continue;
                };
                if let Some(ra) = Self::zone_to_rule_area(
                    &zone,
                    layer_filter,
                    "footprint",
                    fp_kiid.clone(),
                    fp_reference.clone(),
                ) {
                    rule_areas.push(ra);
                }
            }
        }

        Ok(rule_areas)
    }

    /// Collect read-only routing geometry from the active board.
    pub fn get_routing_geometry(&self) -> Result<IpcRoutingGeometry> {
        use kiapi::common::types::KiCadObjectType as T;

        fn class<T: Default>(result: Result<T>) -> IpcGeometryClass<T> {
            match result {
                Ok(items) => IpcGeometryClass::available(items),
                Err(error) => IpcGeometryClass::unavailable(error.to_string()),
            }
        }

        let footprints = class(self.list_footprints());
        let tracks = class(self.get_tracks(None, None));
        let track_arcs = class((|| -> Result<_> {
            let mut arcs = Vec::new();
            for item in self.get_items(T::KotPcbArc)? {
                let arc = kiapi::board::types::Arc::decode(item.value.as_slice())?;
                let (Some(start), Some(mid), Some(end)) =
                    (arc.start.as_ref(), arc.mid.as_ref(), arc.end.as_ref())
                else {
                    anyhow::bail!("track arc item has incomplete IPC geometry")
                };
                arcs.push(IpcTrackArc {
                    net_name: arc.net.as_ref().map(|n| n.name.clone()).unwrap_or_default(),
                    layer: layer_enum_to_name(arc.layer).to_string(),
                    width: arc
                        .width
                        .as_ref()
                        .map(|w| crate::builders::nm_to_mm(w.value_nm))
                        .unwrap_or(0.25),
                    start: IpcVector2 {
                        x: nm_to_mm(start.x_nm),
                        y: nm_to_mm(start.y_nm),
                    },
                    mid: IpcVector2 {
                        x: nm_to_mm(mid.x_nm),
                        y: nm_to_mm(mid.y_nm),
                    },
                    end: IpcVector2 {
                        x: nm_to_mm(end.x_nm),
                        y: nm_to_mm(end.y_nm),
                    },
                });
            }
            Ok(arcs)
        })());
        let footprint_geometry = (|| -> Result<_> {
            let mut pads = Vec::new();
            let mut courtyards = Vec::new();
            for item in self.get_items(T::KotPcbFootprint)? {
                let fp = kiapi::board::types::FootprintInstance::decode(item.value.as_slice())?;
                let reference = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|t| t.text.as_ref())
                    .map(|t| t.text.clone())
                    .unwrap_or_default();
                for child in fp
                    .definition
                    .as_ref()
                    .map(|d| d.items.as_slice())
                    .unwrap_or_default()
                {
                    if child.type_url.ends_with(".Pad") {
                        let pad = kiapi::board::types::Pad::decode(child.value.as_slice())?;
                        let pos = pad.position.as_ref();
                        let stack = pad.pad_stack.as_ref();
                        let copper = stack.and_then(|s| s.copper_layers.first());
                        pads.push(IpcPadGeometry {
                            reference: reference.clone(),
                            number: pad.number,
                            position: IpcVector2 {
                                x: pos.map(|p| nm_to_mm(p.x_nm)).unwrap_or(0.0),
                                y: pos.map(|p| nm_to_mm(p.y_nm)).unwrap_or(0.0),
                            },
                            size: copper.and_then(|c| c.size.as_ref()).map(|s| IpcVector2 {
                                x: nm_to_mm(s.x_nm),
                                y: nm_to_mm(s.y_nm),
                            }),
                            shape: copper.map(|c| {
                                format!(
                                    "{:?}",
                                    kiapi::board::types::PadStackShape::try_from(c.shape)
                                        .unwrap_or(kiapi::board::types::PadStackShape::PssUnknown)
                                )
                            }),
                            rotation: stack
                                .and_then(|s| s.angle.as_ref())
                                .map(|a| a.value_degrees)
                                .unwrap_or(0.0),
                            corner_rounding_ratio: copper.map(|c| c.corner_rounding_ratio),
                            geometry_supported: copper
                                .map(|c| {
                                    matches!(
                                        kiapi::board::types::PadStackShape::try_from(c.shape),
                                        Ok(kiapi::board::types::PadStackShape::PssCircle
                                            | kiapi::board::types::PadStackShape::PssRectangle
                                            | kiapi::board::types::PadStackShape::PssOval
                                            | kiapi::board::types::PadStackShape::PssRoundrect)
                                    )
                                })
                                .unwrap_or(false),
                            geometry_reason: copper.and_then(|c| {
                                let shape =
                                    kiapi::board::types::PadStackShape::try_from(c.shape).ok()?;
                                (!matches!(
                                    shape,
                                    kiapi::board::types::PadStackShape::PssCircle
                                        | kiapi::board::types::PadStackShape::PssRectangle
                                        | kiapi::board::types::PadStackShape::PssOval
                                        | kiapi::board::types::PadStackShape::PssRoundrect
                                ))
                                .then(|| format!("unsupported IPC pad shape: {shape:?}"))
                            }),
                            layers: stack
                                .map(|s| {
                                    s.layers
                                        .iter()
                                        .map(|l| layer_enum_to_name(*l).to_string())
                                        .collect()
                                })
                                .unwrap_or_default(),
                            net_name: pad.net.as_ref().map(|n| n.name.clone()).unwrap_or_default(),
                            net_id: pad
                                .net
                                .as_ref()
                                .and_then(|n| n.code.as_ref())
                                .map(|c| c.value)
                                .unwrap_or(0),
                        });
                    } else if child.type_url.ends_with(".BoardGraphicShape") {
                        let shape =
                            kiapi::board::types::BoardGraphicShape::decode(child.value.as_slice())?;
                        let layer = layer_enum_to_name(shape.layer);
                        if layer == "F.CrtYd" || layer == "B.CrtYd" {
                            if let Some(points) = graphic_shape_points(shape.shape.as_ref()) {
                                courtyards.push(IpcPolygonGeometry {
                                    object_type: "courtyard".into(),
                                    reference: Some(reference.clone()),
                                    layer: layer.into(),
                                    net_name: None,
                                    net_id: None,
                                    polygons: vec![points],
                                });
                            }
                        }
                    }
                }
            }
            Ok((pads, courtyards))
        })();
        let (pads, courtyards) = match footprint_geometry {
            Ok((pads, courtyards)) => (
                IpcGeometryClass::available(pads),
                IpcGeometryClass::available(courtyards),
            ),
            Err(error) => {
                let reason = error.to_string();
                (
                    IpcGeometryClass::unavailable(reason.clone()),
                    IpcGeometryClass::unavailable(reason),
                )
            }
        };

        let vias = class((|| -> Result<_> {
            let mut vias = Vec::new();
            for item in self.get_items(T::KotPcbVia)? {
                let via = kiapi::board::types::Via::decode(item.value.as_slice())?;
                let pos = via.position.as_ref();
                let stack = via.pad_stack.as_ref();
                let copper = stack
                    .and_then(|s| s.copper_layers.first())
                    .and_then(|l| l.size.as_ref());
                let drill = stack
                    .and_then(|s| s.drill.as_ref())
                    .and_then(|d| d.diameter.as_ref());
                vias.push(IpcViaGeometry {
                    kiid: via
                        .id
                        .as_ref()
                        .map(|id| id.value.clone())
                        .unwrap_or_default(),
                    position: IpcVector2 {
                        x: pos.map(|p| nm_to_mm(p.x_nm)).unwrap_or(0.0),
                        y: pos.map(|p| nm_to_mm(p.y_nm)).unwrap_or(0.0),
                    },
                    size: copper.map(|s| IpcVector2 {
                        x: nm_to_mm(s.x_nm),
                        y: nm_to_mm(s.y_nm),
                    }),
                    drill: drill.map(|s| IpcVector2 {
                        x: nm_to_mm(s.x_nm),
                        y: nm_to_mm(s.y_nm),
                    }),
                    layers: stack
                        .map(|s| {
                            s.layers
                                .iter()
                                .map(|l| layer_enum_to_name(*l).to_string())
                                .collect()
                        })
                        .unwrap_or_default(),
                    net_name: via.net.as_ref().map(|n| n.name.clone()).unwrap_or_default(),
                    net_id: via
                        .net
                        .as_ref()
                        .and_then(|n| n.code.as_ref())
                        .map(|c| c.value)
                        .unwrap_or(0),
                });
            }
            Ok(vias)
        })());

        let zones = class((|| -> Result<_> {
            let mut zones = Vec::new();
            for item in self.get_items(T::KotPcbZone)? {
                let zone = kiapi::board::types::Zone::decode(item.value.as_slice())?;
                let (net_name, net_id) = match zone.settings.as_ref() {
                    Some(kiapi::board::types::zone::Settings::CopperSettings(s)) => (
                        Some(s.net.as_ref().map(|n| n.name.clone()).unwrap_or_default()),
                        Some(
                            s.net
                                .as_ref()
                                .and_then(|n| n.code.as_ref())
                                .map(|c| c.value)
                                .unwrap_or(0),
                        ),
                    ),
                    _ => (None, None),
                };
                let polygons = polyset_points(zone.outline.as_ref());
                for layer in &zone.layers {
                    zones.push(IpcPolygonGeometry {
                        object_type: "zone_outline".into(),
                        reference: None,
                        layer: layer_enum_to_name(*layer).into(),
                        net_name: net_name.clone(),
                        net_id,
                        polygons: polygons.clone(),
                    });
                }
            }
            Ok(zones)
        })());

        let board_edges = class((|| -> Result<_> {
            use kiapi::common::types::graphic_shape::Geometry;
            let mut edges = Vec::new();
            for item in self.get_items(T::KotPcbShape)? {
                let graphic =
                    kiapi::board::types::BoardGraphicShape::decode(item.value.as_slice())?;
                if layer_enum_to_name(graphic.layer) != "Edge.Cuts" {
                    continue;
                }
                match graphic.shape.and_then(|s| s.geometry) {
                    Some(Geometry::Segment(s)) => {
                        if let (Some(a), Some(b)) = (s.start, s.end) {
                            edges.push(IpcBoardEdgeGeometry::Line {
                                start: point(&a),
                                end: point(&b),
                            });
                        }
                    }
                    Some(Geometry::Arc(a)) => {
                        if let (Some(start), Some(mid), Some(end)) = (a.start, a.mid, a.end) {
                            edges.push(IpcBoardEdgeGeometry::Arc {
                                start: point(&start),
                                mid: point(&mid),
                                end: point(&end),
                            });
                        }
                    }
                    Some(_) => {
                        anyhow::bail!("unsupported Edge.Cuts primitive returned by KiCad IPC")
                    }
                    None => anyhow::bail!("Edge.Cuts item has no IPC geometry"),
                }
            }
            Ok(edges)
        })());

        Ok(IpcRoutingGeometry {
            layers: class(self.get_layers()),
            board_extents: match self.get_board_extents() {
                Ok(extents) => IpcGeometryClass::available(Some(extents)),
                Err(error) => IpcGeometryClass::unavailable(error.to_string()),
            },
            footprints,
            pads,
            tracks,
            track_arcs,
            vias,
            courtyards,
            zones,
            board_edges,
        })
    }

    /// Move a footprint to a new position.
    pub fn move_footprint(&self, reference: &str, x: f64, y: f64) -> Result<()> {
        // Find the footprint, update position, send UpdateItems
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(mut fp) =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    let old = fp.position.unwrap_or_default();
                    let new_pos = crate::builders::vec2(x, y);
                    fp.position = Some(new_pos);
                    // KiCAD carries the footprint's children (pads, silk,
                    // text) in absolute board coordinates and re-creates them
                    // verbatim on update, so they must be shifted along with
                    // the anchor (issue #23).
                    crate::transform::transform_footprint_children(
                        &mut fp,
                        &crate::transform::Xform::Translate {
                            dx_nm: new_pos.x_nm - old.x_nm,
                            dy_nm: new_pos.y_nm - old.y_nm,
                        },
                    )?;
                    let any = crate::builders::pack_any(&fp, "kiapi.board.types.FootprintInstance");
                    self.update_items(vec![any])?;
                    return Ok(());
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found", reference)
    }

    /// Rotate a footprint to a new angle.
    pub fn rotate_footprint(&self, reference: &str, angle: f64) -> Result<()> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in &items {
            if let Ok(mut fp) =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            {
                let ref_text = fp
                    .reference_field
                    .as_ref()
                    .and_then(|f| f.text.as_ref())
                    .and_then(|bt| bt.text.as_ref())
                    .map(|t| t.text.as_str())
                    .unwrap_or("");
                if ref_text == reference {
                    let old_deg = fp
                        .orientation
                        .as_ref()
                        .map(|a| a.value_degrees)
                        .unwrap_or(0.0);
                    fp.orientation = Some(kiapi::common::types::Angle {
                        value_degrees: angle,
                    });
                    // Children are carried in absolute board coordinates and
                    // angles; rotate them around the anchor like KiCAD's
                    // FOOTPRINT::SetOrientation does natively (issue #23).
                    let anchor = fp.position.unwrap_or_default();
                    crate::transform::transform_footprint_children(
                        &mut fp,
                        &crate::transform::Xform::Rotate {
                            cx_nm: anchor.x_nm,
                            cy_nm: anchor.y_nm,
                            delta_deg: angle - old_deg,
                        },
                    )?;
                    let any = crate::builders::pack_any(&fp, "kiapi.board.types.FootprintInstance");
                    self.update_items(vec![any])?;
                    return Ok(());
                }
            }
        }
        anyhow::bail!("Footprint '{}' not found", reference)
    }

    /// Update the visible value field of an existing footprint.
    pub fn set_footprint_value(&self, reference: &str, value: &str) -> Result<()> {
        let items = self.get_items(kiapi::common::types::KiCadObjectType::KotPcbFootprint)?;
        for item in items {
            if let Ok(mut footprint) =
                kiapi::board::types::FootprintInstance::decode(item.value.as_slice())
            {
                let current_reference = footprint
                    .reference_field
                    .as_ref()
                    .and_then(|field| field.text.as_ref())
                    .and_then(|board_text| board_text.text.as_ref())
                    .map(|text| text.text.as_str())
                    .unwrap_or("");
                if current_reference != reference {
                    continue;
                }
                if let Some(text) = footprint
                    .value_field
                    .as_mut()
                    .and_then(|field| field.text.as_mut())
                    .and_then(|board_text| board_text.text.as_mut())
                {
                    text.text = value.to_string();
                } else {
                    anyhow::bail!("Footprint '{reference}' has no editable value field");
                }
                if let Some(text) = footprint
                    .definition
                    .as_mut()
                    .and_then(|definition| definition.value_field.as_mut())
                    .and_then(|field| field.text.as_mut())
                    .and_then(|board_text| board_text.text.as_mut())
                {
                    text.text = value.to_string();
                }
                let any =
                    crate::builders::pack_any(&footprint, "kiapi.board.types.FootprintInstance");
                self.update_items(vec![any])?;
                return Ok(());
            }
        }
        anyhow::bail!("Footprint '{reference}' not found")
    }

    /// Delete a footprint by reference.
    pub fn delete_footprint(&self, reference: &str) -> Result<()> {
        let kiid = self.find_footprint_kiid(reference)?;
        self.delete_items(vec![kiid])?;
        // KiCad returns no per-item deletion results (#116), so the response
        // alone cannot distinguish "deleted" from "silently skipped". Ask the
        // board instead — reporting success without evidence is the failure
        // mode #99 was closed to eliminate.
        match self.find_footprint_kiid(reference) {
            Err(_) => Ok(()),
            Ok(_) => anyhow::bail!(
                "KiCad reported no error but footprint '{reference}' is still on the board"
            ),
        }
    }

    /// Build a typed footprint item suitable for [`Self::create_items`].
    ///
    /// Children (pads and graphics) are emitted in ABSOLUTE board
    /// coordinates: KiCAD serializes `FootprintInstance` children that way
    /// and re-creates them verbatim (issue #23 — see the `transform` module
    /// docs), so every footprint-local point is rotated and translated here.
    #[allow(clippy::too_many_arguments)]
    pub fn build_footprint_item(
        &self,
        lib_id: &str,
        reference: &str,
        value: &str,
        pads: &[IpcPadDefinition],
        graphics: &[IpcGraphicDefinition],
        fields: &IpcFieldPlacement,
        x: f64,
        y: f64,
        rotation: f64,
        layer: &str,
    ) -> Result<prost_types::Any> {
        let (library_nickname, entry_name) = lib_id
            .split_once(':')
            .context("footprint identifier must use Library:Footprint syntax")?;
        let text_field = |name: &str, text: &str, local: (f64, f64, f64), visible: bool| {
            // Field text positions come footprint-local from the library and
            // are transformed exactly like pads, so the placed part keeps the
            // library's text layout instead of a synthesized offset that can
            // sit on the part's own silkscreen.
            let (fx, fy, frot) = local;
            let (bx, by) = konnect_sexp::geometry::transform_pad(fx, fy, x, y, rotation);
            kiapi::board::types::Field {
                id: None,
                name: name.to_string(),
                text: Some(kiapi::board::types::BoardText {
                    id: None,
                    text: Some(kiapi::common::types::Text {
                        position: Some(crate::builders::vec2(bx, by)),
                        attributes: Some(kiapi::common::types::TextAttributes {
                            size: Some(crate::builders::vec2(1.0, 1.0)),
                            angle: Some(kiapi::common::types::Angle {
                                value_degrees: readable_text_angle(rotation + frot),
                            }),
                            ..Default::default()
                        }),
                        text: text.to_string(),
                        hyperlink: String::new(),
                    }),
                    layer: crate::builders::layer_from_name(if layer == "B.Cu" {
                        "B.SilkS"
                    } else {
                        "F.SilkS"
                    }) as i32,
                    knockout: false,
                    locked: kiapi::common::types::LockedState::LsUnlocked as i32,
                }),
                visible,
            }
        };
        let reference_field = text_field(
            "Reference",
            reference,
            fields.reference_at.unwrap_or((0.0, -1.0, 0.0)),
            true,
        );
        let value_field = text_field(
            "Value",
            value,
            fields.value_at.unwrap_or((0.0, 1.0, 0.0)),
            false,
        );
        let mut child_items: Vec<prost_types::Any> = pads
            .iter()
            .map(|pad| {
                // Canonical KiCAD footprint-local → board transform; see
                // konnect_sexp::geometry::transform_pad for why the sin terms
                // are not the textbook rotation matrix (Y axis points down).
                let (board_x, board_y) =
                    konnect_sexp::geometry::transform_pad(pad.x, pad.y, x, y, rotation);
                let mut layers = Vec::new();
                for name in &pad.layers {
                    match name.as_str() {
                        "*.Cu" => layers.extend(3..=34),
                        "*.Mask" => layers.extend([
                            kiapi::board::types::BoardLayer::BlFMask as i32,
                            kiapi::board::types::BoardLayer::BlBMask as i32,
                        ]),
                        "*.Paste" => layers.extend([
                            kiapi::board::types::BoardLayer::BlFPaste as i32,
                            kiapi::board::types::BoardLayer::BlBPaste as i32,
                        ]),
                        name => layers.push(crate::builders::layer_from_name(name) as i32),
                    }
                }
                layers
                    .retain(|layer| *layer != kiapi::board::types::BoardLayer::BlUndefined as i32);
                layers.sort_unstable();
                layers.dedup();

                let shape = match pad.shape.as_str() {
                    "circle" => kiapi::board::types::PadStackShape::PssCircle,
                    "rect" => kiapi::board::types::PadStackShape::PssRectangle,
                    "oval" => kiapi::board::types::PadStackShape::PssOval,
                    "trapezoid" => kiapi::board::types::PadStackShape::PssTrapezoid,
                    "roundrect" => kiapi::board::types::PadStackShape::PssRoundrect,
                    "chamfered_rect" => kiapi::board::types::PadStackShape::PssChamferedrect,
                    _ => kiapi::board::types::PadStackShape::PssRectangle,
                };
                // Always F_Cu: it is KiCad's ALL_LAYERS sentinel for a
                // PST_NORMAL stack, not a statement about which side the pad
                // is on (that is `layers`). PADSTACK::unpackCopperLayer
                // rejects any other value while the mode is NORMAL, failing
                // the whole deserialization — see #117.
                let copper = kiapi::board::types::PadStackLayer {
                    layer: kiapi::board::types::BoardLayer::BlFCu as i32,
                    shape: shape as i32,
                    size: Some(crate::builders::vec2(pad.size_x, pad.size_y)),
                    corner_rounding_ratio: pad.roundrect_ratio,
                    custom_anchor_shape: shape as i32,
                    offset: Some(crate::builders::vec2(0.0, 0.0)),
                    ..Default::default()
                };
                let drill = pad
                    .drill_x
                    .map(|drill_x| kiapi::board::types::DrillProperties {
                        start_layer: kiapi::board::types::BoardLayer::BlFCu as i32,
                        end_layer: kiapi::board::types::BoardLayer::BlBCu as i32,
                        diameter: Some(crate::builders::vec2(
                            drill_x,
                            pad.drill_y.unwrap_or(drill_x),
                        )),
                        shape: if pad.drill_oval {
                            kiapi::board::types::DrillShape::DsOblong as i32
                        } else {
                            kiapi::board::types::DrillShape::DsCircle as i32
                        },
                        ..Default::default()
                    });
                let stack = kiapi::board::types::PadStack {
                    r#type: kiapi::board::types::PadStackType::PstNormal as i32,
                    layers,
                    drill,
                    unconnected_layer_removal: kiapi::board::types::UnconnectedLayerRemoval::UlrKeep
                        as i32,
                    copper_layers: vec![copper],
                    angle: Some(kiapi::common::types::Angle {
                        value_degrees: rotation + pad.rotation,
                    }),
                    ..Default::default()
                };
                let pad_type = match pad.pad_type.as_str() {
                    "thru_hole" => kiapi::board::types::PadType::PtPth,
                    "np_thru_hole" => kiapi::board::types::PadType::PtNpth,
                    "connect" => kiapi::board::types::PadType::PtEdgeConnector,
                    _ => kiapi::board::types::PadType::PtSmd,
                };
                let item = kiapi::board::types::Pad {
                    number: pad.number.clone(),
                    r#type: pad_type as i32,
                    pad_stack: Some(stack),
                    position: Some(crate::builders::vec2(board_x, board_y)),
                    locked: kiapi::common::types::LockedState::LsUnlocked as i32,
                    ..Default::default()
                };
                crate::builders::pack_any(&item, "kiapi.board.types.Pad")
            })
            .collect();
        child_items.extend(
            graphics
                .iter()
                .map(|graphic| build_graphic_child(graphic, x, y, rotation)),
        );
        let definition = kiapi::board::types::Footprint {
            id: Some(kiapi::common::types::LibraryIdentifier {
                library_nickname: library_nickname.to_string(),
                entry_name: entry_name.to_string(),
            }),
            reference_field: Some(reference_field.clone()),
            value_field: Some(value_field.clone()),
            items: child_items,
            ..Default::default()
        };
        let footprint = kiapi::board::types::FootprintInstance {
            position: Some(crate::builders::vec2(x, y)),
            orientation: Some(kiapi::common::types::Angle {
                value_degrees: rotation,
            }),
            layer: crate::builders::layer_from_name(layer) as i32,
            locked: kiapi::common::types::LockedState::LsUnlocked as i32,
            definition: Some(definition),
            reference_field: Some(reference_field),
            value_field: Some(value_field),
            ..Default::default()
        };
        Ok(crate::builders::pack_any(
            &footprint,
            "kiapi.board.types.FootprintInstance",
        ))
    }

    /// Place and verify a footprint instance through the typed KiCad API.
    #[allow(clippy::too_many_arguments)]
    pub fn place_footprint(
        &self,
        board: &Path,
        lib_id: &str,
        reference: &str,
        value: &str,
        pads: &[IpcPadDefinition],
        graphics: &[IpcGraphicDefinition],
        fields: &IpcFieldPlacement,
        x: f64,
        y: f64,
        rotation: f64,
        layer: &str,
    ) -> Result<IpcFootprint> {
        // Target the document matching `board`, not whichever board is first
        // in KiCad's open list — with several boards open, first-document
        // targeting mutates the wrong one (caught in live verification).
        let document = self.find_open_board(board)?;
        if self
            .list_footprints_in(document.clone())?
            .iter()
            .any(|footprint| footprint.reference == reference)
        {
            anyhow::bail!("footprint reference '{reference}' already exists on the board");
        }
        let item = self.build_footprint_item(
            lib_id, reference, value, pads, graphics, fields, x, y, rotation, layer,
        )?;
        self.create_items_in(document.clone(), vec![item])?;
        let footprints = self.list_footprints_in(document)?;
        footprints
            .iter()
            .find(|footprint| footprint.reference == reference)
            .cloned()
            .with_context(|| {
                let references = footprints
                    .iter()
                    .map(|footprint| footprint.reference.as_str())
                    .collect::<Vec<_>>()
                    .join(", ");
                format!(
                    "KiCad created the footprint but reference '{reference}' was not found (board references: {references})"
                )
            })
    }

    /// Get board extents (bounding box of all items).
    pub fn get_board_extents(&self) -> Result<IpcBoardExtents> {
        // Use GetBoundingBox with no specific items = board extents
        let header = self.make_header()?;
        let cmd = kiapi::common::commands::GetBoundingBox {
            header: Some(header),
            items: vec![], // empty = all items
            mode: kiapi::common::commands::BoundingBoxMode::BbmItemOnly as i32,
        };
        let resp_any = self.send_command(&cmd, "kiapi.common.commands.GetBoundingBox")?;
        if let Some(any) = resp_any {
            let resp: kiapi::common::commands::GetBoundingBoxResponse = unpack_any(&any)?;
            if let Some(bbox) = resp.boxes.first() {
                let pos = bbox.position.as_ref();
                let size = bbox.size.as_ref();
                return Ok(IpcBoardExtents {
                    min: IpcVector2 {
                        x: pos
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0),
                        y: pos
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0),
                    },
                    max: IpcVector2 {
                        x: pos
                            .map(|p| crate::builders::nm_to_mm(p.x_nm))
                            .unwrap_or(0.0)
                            + size
                                .map(|s| crate::builders::nm_to_mm(s.x_nm))
                                .unwrap_or(0.0),
                        y: pos
                            .map(|p| crate::builders::nm_to_mm(p.y_nm))
                            .unwrap_or(0.0)
                            + size
                                .map(|s| crate::builders::nm_to_mm(s.y_nm))
                                .unwrap_or(0.0),
                    },
                });
            }
        }
        anyhow::bail!("No bounding box returned from KiCAD")
    }

    /// Get enabled layers.
    pub fn get_layers(&self) -> Result<Vec<IpcLayer>> {
        let doc = self.get_board_document()?;
        let cmd = kiapi::board::commands::GetBoardEnabledLayers { board: Some(doc) };
        let resp_any = self.send_command(&cmd, "kiapi.board.commands.GetBoardEnabledLayers")?;
        if let Some(any) = resp_any {
            let resp: kiapi::board::commands::BoardEnabledLayersResponse = unpack_any(&any)?;
            let layers = resp
                .layers
                .iter()
                .map(|&l| {
                    let bl = kiapi::board::types::BoardLayer::try_from(l)
                        .unwrap_or(kiapi::board::types::BoardLayer::BlUndefined);
                    IpcLayer {
                        name: bl
                            .as_str_name()
                            .trim_start_matches("BL_")
                            .replace('_', ".")
                            .to_string(),
                        id: l,
                        kind: String::new(),
                    }
                })
                .collect();
            Ok(layers)
        } else {
            Ok(vec![])
        }
    }

    /// Run an arbitrary tool action in KiCAD (e.g. to trigger a refresh).
    pub fn run_action(&self, action: &str) -> Result<()> {
        let cmd = kiapi::common::commands::RunAction {
            action: action.to_string(),
        };
        self.send_command(&cmd, "kiapi.common.commands.RunAction")?;
        Ok(())
    }
}

/// True when `rotation` keeps axis-aligned shapes axis-aligned.
fn is_cardinal_rotation(rotation_deg: f64) -> bool {
    let r = rotation_deg.rem_euclid(90.0);
    r.abs() < 1e-9 || (90.0 - r).abs() < 1e-9
}

/// Normalize a text angle the way KiCAD keeps labels readable: fold into
/// [0, 360), then flip anything that would read upside down by 180°.
fn readable_text_angle(deg: f64) -> f64 {
    let mut angle = deg.rem_euclid(360.0);
    if angle > 90.0 && angle <= 270.0 {
        angle -= 180.0;
    }
    angle
}

/// Transform one footprint-local graphic into an absolute-board-space child
/// item for a `FootprintInstance` (see [`KiCadIpcClient::build_footprint_item`]).
fn build_graphic_child(
    graphic: &IpcGraphicDefinition,
    x: f64,
    y: f64,
    rotation: f64,
) -> prost_types::Any {
    use crate::builders;
    const SHAPE: &str = "kiapi.board.types.BoardGraphicShape";
    let xf = |(px, py): (f64, f64)| konnect_sexp::geometry::transform_pad(px, py, x, y, rotation);
    match graphic {
        IpcGraphicDefinition::Line {
            start,
            end,
            layer,
            width,
        } => {
            let (x1, y1) = xf(*start);
            let (x2, y2) = xf(*end);
            builders::pack_any(
                &builders::board_segment(layer, *width, x1, y1, x2, y2),
                SHAPE,
            )
        }
        IpcGraphicDefinition::Rect {
            start,
            end,
            layer,
            width,
            filled,
        } => {
            if is_cardinal_rotation(rotation) {
                // A 90°-multiple keeps the rectangle axis-aligned; rotate the
                // corners and re-normalize which one is top-left.
                let (x1, y1) = xf(*start);
                let (x2, y2) = xf(*end);
                builders::pack_any(
                    &builders::board_rectangle(
                        layer,
                        *width,
                        x1.min(x2),
                        y1.min(y2),
                        x1.max(x2),
                        y1.max(y2),
                        *filled,
                    ),
                    SHAPE,
                )
            } else {
                // The Rectangle message is axis-aligned by construction, so a
                // non-cardinal rotation emits the four rotated corners as a
                // polygon — the same degradation EDA_SHAPE::Rotate applies.
                let corners = vec![
                    xf(*start),
                    xf((end.0, start.1)),
                    xf(*end),
                    xf((start.0, end.1)),
                ];
                builders::pack_any(
                    &builders::board_polygon(layer, *width, *filled, &[corners]),
                    SHAPE,
                )
            }
        }
        IpcGraphicDefinition::Circle {
            center,
            end,
            layer,
            width,
            filled,
        } => {
            let (cx, cy) = xf(*center);
            // The radius is rotation-invariant; keep KiCAD's center +
            // circumference-point encoding by re-deriving it from the length.
            let radius = ((end.0 - center.0).powi(2) + (end.1 - center.1).powi(2)).sqrt();
            builders::pack_any(
                &builders::board_circle(layer, *width, cx, cy, radius, *filled),
                SHAPE,
            )
        }
        IpcGraphicDefinition::Arc {
            start,
            mid,
            end,
            layer,
            width,
        } => {
            let (sx, sy) = xf(*start);
            let (mx, my) = xf(*mid);
            let (ex, ey) = xf(*end);
            builders::pack_any(
                &builders::board_arc(layer, *width, sx, sy, mx, my, ex, ey),
                SHAPE,
            )
        }
        IpcGraphicDefinition::Poly {
            points,
            layer,
            width,
            filled,
        } => {
            let transformed: Vec<(f64, f64)> = points.iter().map(|p| xf(*p)).collect();
            builders::pack_any(
                &builders::board_polygon(layer, *width, *filled, &[transformed]),
                SHAPE,
            )
        }
        IpcGraphicDefinition::Text {
            text,
            position,
            rotation: text_rotation,
            layer,
            size,
        } => {
            let (tx, ty) = xf(*position);
            builders::pack_any(
                &builders::board_text(
                    layer,
                    text,
                    tx,
                    ty,
                    *size,
                    readable_text_angle(text_rotation + rotation),
                    false,
                ),
                "kiapi.board.types.BoardText",
            )
        }
    }
}

fn header_for(
    document: kiapi::common::types::DocumentSpecifier,
) -> kiapi::common::types::ItemHeader {
    kiapi::common::types::ItemHeader {
        document: Some(document),
        container: None,
        field_mask: None,
    }
}

fn board_document_path(document: &kiapi::common::types::DocumentSpecifier) -> Option<PathBuf> {
    use kiapi::common::types::document_specifier::Identifier;

    let Identifier::BoardFilename(filename) = document.identifier.as_ref()? else {
        return None;
    };
    let path = PathBuf::from(filename);
    if path.is_absolute() {
        return Some(path);
    }
    document
        .project
        .as_ref()
        .filter(|project| !project.path.is_empty())
        .map(|project| PathBuf::from(&project.path).join(&path))
        .or(Some(path))
}

fn paths_refer_to_same_board(requested: &Path, active: &Path) -> bool {
    match (requested.canonicalize(), active.canonicalize()) {
        (Ok(requested), Ok(active)) => requested == active,
        _ if active.components().count() == 1 => {
            requested.components().count() == 1 && requested.file_name() == active.file_name()
        }
        _ => requested == active,
    }
}

#[cfg(test)]
mod document_path_tests {
    use super::*;

    #[test]
    fn relative_board_filename_is_resolved_against_project_path() {
        let document = kiapi::common::types::DocumentSpecifier {
            r#type: kiapi::common::types::DocumentType::DoctypePcb as i32,
            identifier: Some(
                kiapi::common::types::document_specifier::Identifier::BoardFilename(
                    "controller.kicad_pcb".to_string(),
                ),
            ),
            project: Some(kiapi::common::types::ProjectSpecifier {
                name: "controller".to_string(),
                path: "/work/controller".to_string(),
            }),
        };

        assert_eq!(
            board_document_path(&document).unwrap(),
            PathBuf::from("/work/controller/controller.kicad_pcb")
        );
    }

    #[test]
    fn bare_kicad_filename_is_not_enough_to_authorize_an_absolute_request() {
        assert!(!paths_refer_to_same_board(
            Path::new("/work/controller/controller.kicad_pcb"),
            Path::new("controller.kicad_pcb")
        ));
        assert!(paths_refer_to_same_board(
            Path::new("controller.kicad_pcb"),
            Path::new("controller.kicad_pcb")
        ));
        assert!(!paths_refer_to_same_board(
            Path::new("/work/controller/other.kicad_pcb"),
            Path::new("controller.kicad_pcb")
        ));
    }
}

#[cfg(test)]
mod rule_area_tests {
    use super::*;

    fn rule_area_zone(
        keepout_tracks: bool,
        keepout_vias: bool,
        keepout_copper: bool,
        keepout_pads: bool,
        keepout_footprints: bool,
    ) -> kiapi::board::types::Zone {
        kiapi::board::types::Zone {
            id: Some(kiapi::common::types::Kiid {
                value: "11111111-1111-1111-1111-111111111111".into(),
            }),
            r#type: kiapi::board::types::ZoneType::ZtRuleArea as i32,
            layers: vec![kiapi::board::types::BoardLayer::BlFCu as i32],
            outline: Some(kiapi::common::types::PolySet {
                polygons: vec![kiapi::common::types::PolygonWithHoles {
                    outline: Some(kiapi::common::types::PolyLine {
                        nodes: vec![node(0, 0), node(1_000_000, 0), node(1_000_000, 1_000_000)],
                        closed: true,
                    }),
                    holes: vec![],
                }],
            }),
            name: "TestKeepout".into(),
            settings: Some(kiapi::board::types::zone::Settings::RuleAreaSettings(
                kiapi::board::types::RuleAreaSettings {
                    keepout_copper,
                    keepout_vias,
                    keepout_tracks,
                    keepout_pads,
                    keepout_footprints,
                    placement_enabled: false,
                    placement_source_type: 0,
                    placement_source: String::new(),
                },
            )),
            priority: 0,
            filled: false,
            filled_polygons: vec![],
            border: None,
            locked: kiapi::common::types::LockedState::LsUnlocked as i32,
            layer_properties: vec![],
        }
    }

    fn node(x_nm: i64, y_nm: i64) -> kiapi::common::types::PolyLineNode {
        kiapi::common::types::PolyLineNode {
            geometry: Some(kiapi::common::types::poly_line_node::Geometry::Point(
                kiapi::common::types::Vector2 { x_nm, y_nm },
            )),
        }
    }

    fn copper_zone() -> kiapi::board::types::Zone {
        let mut z = rule_area_zone(false, false, false, false, false);
        z.r#type = kiapi::board::types::ZoneType::ZtCopper as i32;
        z.settings = Some(kiapi::board::types::zone::Settings::CopperSettings(
            kiapi::board::types::CopperZoneSettings::default(),
        ));
        z
    }

    #[test]
    fn ordinary_copper_zone_is_excluded() {
        let zone = copper_zone();
        assert!(
            KiCadIpcClient::zone_to_rule_area(&zone, None, "board", None, None).is_none(),
            "a CopperZoneSettings zone must never be reported as a rule area"
        );
    }

    #[test]
    fn rule_area_is_included_with_expected_fields() {
        let zone = rule_area_zone(true, true, false, false, true);
        let ra = KiCadIpcClient::zone_to_rule_area(&zone, None, "board", None, None)
            .expect("RuleAreaSettings zone must be included");
        assert_eq!(ra.kiid, "11111111-1111-1111-1111-111111111111");
        assert_eq!(ra.name, "TestKeepout");
        assert_eq!(ra.layers, vec!["F.Cu".to_string()]);
        assert!(ra.keepout_tracks);
        assert!(ra.keepout_vias);
        assert!(!ra.keepout_copper);
        assert!(!ra.keepout_pads);
        assert!(ra.keepout_footprints);
        assert_eq!(ra.owner, "board");
        assert!(ra.footprint_kiid.is_none());
    }

    #[test]
    fn geometry_bounds_are_preserved() {
        let zone = rule_area_zone(true, false, false, false, false);
        let ra = KiCadIpcClient::zone_to_rule_area(&zone, None, "board", None, None).unwrap();
        assert_eq!(ra.bounds.len(), 1);
        assert_eq!(ra.bounds[0].len(), 3);
        // nm -> mm conversion applied, same helper used by tracks/vias.
        assert!((ra.bounds[0][1].x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn layer_filter_matching_layer_is_included() {
        let zone = rule_area_zone(true, false, false, false, false);
        assert!(
            KiCadIpcClient::zone_to_rule_area(&zone, Some("F.Cu"), "board", None, None).is_some()
        );
    }

    #[test]
    fn layer_filter_non_matching_layer_is_excluded() {
        let zone = rule_area_zone(true, false, false, false, false);
        assert!(
            KiCadIpcClient::zone_to_rule_area(&zone, Some("B.Cu"), "board", None, None).is_none()
        );
    }

    #[test]
    fn footprint_owner_fields_are_attached() {
        let zone = rule_area_zone(false, false, false, false, false);
        let ra = KiCadIpcClient::zone_to_rule_area(
            &zone,
            None,
            "footprint",
            Some("fp-kiid-1".to_string()),
            Some("U1".to_string()),
        )
        .unwrap();
        assert_eq!(ra.owner, "footprint");
        assert_eq!(ra.footprint_kiid.as_deref(), Some("fp-kiid-1"));
        assert_eq!(ra.footprint_reference.as_deref(), Some("U1"));
    }

    #[test]
    fn zone_with_no_settings_is_excluded() {
        let mut zone = rule_area_zone(true, false, false, false, false);
        zone.settings = None;
        assert!(KiCadIpcClient::zone_to_rule_area(&zone, None, "board", None, None).is_none());
    }
}

#[cfg(test)]
mod footprint_graphics_tests {
    use super::*;
    use prost::Message;

    fn build(
        graphics: &[IpcGraphicDefinition],
        x: f64,
        y: f64,
        rotation: f64,
    ) -> kiapi::board::types::FootprintInstance {
        // build_footprint_item is pure — no IPC round-trip — so the socket
        // path is never dialed.
        let client = KiCadIpcClient::new("tcp://never-dialed");
        let any = client
            .build_footprint_item(
                "Lib:Fp",
                "R1",
                "R",
                &[],
                graphics,
                &crate::types::IpcFieldPlacement::default(),
                x,
                y,
                rotation,
                "F.Cu",
            )
            .unwrap();
        kiapi::board::types::FootprintInstance::decode(any.value.as_slice()).unwrap()
    }

    fn shapes(
        fp: &kiapi::board::types::FootprintInstance,
    ) -> Vec<kiapi::board::types::BoardGraphicShape> {
        fp.definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|any| any.type_url.ends_with("BoardGraphicShape"))
            .map(|any| {
                kiapi::board::types::BoardGraphicShape::decode(any.value.as_slice()).unwrap()
            })
            .collect()
    }

    fn texts(fp: &kiapi::board::types::FootprintInstance) -> Vec<kiapi::board::types::BoardText> {
        fp.definition
            .as_ref()
            .unwrap()
            .items
            .iter()
            .filter(|any| any.type_url.ends_with("BoardText"))
            .map(|any| kiapi::board::types::BoardText::decode(any.value.as_slice()).unwrap())
            .collect()
    }

    /// Courtyard rect + silk line + fab text at rotation 90 must come out on
    /// their own layers, at absolute board coordinates rotated with the part.
    ///
    /// transform_pad at 90°: (lx, ly) → (x + ly, y − lx).
    #[test]
    fn rotation_90_pretransforms_graphics_to_absolute_board_space() {
        let graphics = vec![
            IpcGraphicDefinition::Rect {
                start: (-1.0, -2.0),
                end: (1.0, 2.0),
                layer: "F.CrtYd".to_string(),
                width: 0.05,
                filled: false,
            },
            IpcGraphicDefinition::Line {
                start: (-1.0, 0.0),
                end: (1.0, 0.0),
                layer: "F.SilkS".to_string(),
                width: 0.12,
            },
            IpcGraphicDefinition::Text {
                text: "hello".to_string(),
                position: (0.0, 2.0),
                rotation: 0.0,
                layer: "F.Fab".to_string(),
                size: 0.5,
            },
        ];
        let fp = build(&graphics, 100.0, 50.0, 90.0);

        let shapes = shapes(&fp);
        assert_eq!(shapes.len(), 2);

        // The rectangle stays axis-aligned at a cardinal rotation: corners
        // (-1,-2) → (98, 51) and (1,2) → (102, 49), re-normalized so the
        // top-left really is top-left.
        let rect = &shapes[0];
        assert_eq!(rect.layer, kiapi::board::types::BoardLayer::BlFCrtYd as i32);
        let geometry = rect.shape.as_ref().unwrap().geometry.as_ref().unwrap();
        let kiapi::common::types::graphic_shape::Geometry::Rectangle(r) = geometry else {
            panic!("expected Rectangle geometry, got {geometry:?}");
        };
        assert_eq!(r.top_left.unwrap().x_nm, 98_000_000);
        assert_eq!(r.top_left.unwrap().y_nm, 49_000_000);
        assert_eq!(r.bottom_right.unwrap().x_nm, 102_000_000);
        assert_eq!(r.bottom_right.unwrap().y_nm, 51_000_000);

        // Silk line (-1,0)→(1,0) rotates to (100,51)→(100,49).
        let line = &shapes[1];
        assert_eq!(line.layer, kiapi::board::types::BoardLayer::BlFSilkS as i32);
        let geometry = line.shape.as_ref().unwrap().geometry.as_ref().unwrap();
        let kiapi::common::types::graphic_shape::Geometry::Segment(s) = geometry else {
            panic!("expected Segment geometry, got {geometry:?}");
        };
        assert_eq!(s.start.unwrap().x_nm, 100_000_000);
        assert_eq!(s.start.unwrap().y_nm, 51_000_000);
        assert_eq!(s.end.unwrap().x_nm, 100_000_000);
        assert_eq!(s.end.unwrap().y_nm, 49_000_000);

        // Fab text at local (0,2) lands at (102, 50) with its angle rotated.
        let texts = texts(&fp);
        assert_eq!(texts.len(), 1);
        let text = texts[0].text.as_ref().unwrap();
        assert_eq!(
            texts[0].layer,
            kiapi::board::types::BoardLayer::BlFFab as i32
        );
        assert_eq!(text.position.unwrap().x_nm, 102_000_000);
        assert_eq!(text.position.unwrap().y_nm, 50_000_000);
        assert_eq!(
            text.attributes
                .as_ref()
                .unwrap()
                .angle
                .unwrap()
                .value_degrees,
            90.0
        );
    }

    /// At 180° the text would read upside down; KiCAD keeps labels readable by
    /// flipping such angles 180°, and so does the emitted child.
    #[test]
    fn text_angles_are_kept_readable() {
        let graphics = vec![IpcGraphicDefinition::Text {
            text: "hello".to_string(),
            position: (0.0, 0.0),
            rotation: 0.0,
            layer: "F.Fab".to_string(),
            size: 0.5,
        }];
        let fp = build(&graphics, 0.0, 0.0, 180.0);
        let texts = texts(&fp);
        assert_eq!(
            texts[0]
                .text
                .as_ref()
                .unwrap()
                .attributes
                .as_ref()
                .unwrap()
                .angle
                .unwrap()
                .value_degrees,
            0.0
        );
    }

    /// The Rectangle protobuf message is axis-aligned by construction, so a
    /// non-cardinal rotation must degrade the rect to its four rotated corners
    /// as a polygon — mirroring EDA_SHAPE::Rotate.
    #[test]
    fn non_cardinal_rotation_emits_rect_as_polygon() {
        let graphics = vec![IpcGraphicDefinition::Rect {
            start: (-1.0, -1.0),
            end: (1.0, 1.0),
            layer: "F.CrtYd".to_string(),
            width: 0.05,
            filled: false,
        }];
        let fp = build(&graphics, 0.0, 0.0, 45.0);
        let shapes = shapes(&fp);
        assert_eq!(shapes.len(), 1);
        let geometry = shapes[0].shape.as_ref().unwrap().geometry.as_ref().unwrap();
        let kiapi::common::types::graphic_shape::Geometry::Polygon(poly) = geometry else {
            panic!("expected Polygon geometry for a 45-degree rect, got {geometry:?}");
        };
        let outline = poly.polygons[0].outline.as_ref().unwrap();
        assert!(outline.closed);
        assert_eq!(outline.nodes.len(), 4);
        // Corner (-1,-1) at 45°: bx = -cos45 - sin45 = -√2, by = sin45 - cos45 = 0.
        let first = match outline.nodes[0].geometry.as_ref().unwrap() {
            kiapi::common::types::poly_line_node::Geometry::Point(p) => p,
            other => panic!("expected Point node, got {other:?}"),
        };
        let sqrt2_nm = (std::f64::consts::SQRT_2 * 1_000_000.0) as i64;
        assert!((first.x_nm + sqrt2_nm).abs() < 10, "x={}", first.x_nm);
        assert!(first.y_nm.abs() < 10, "y={}", first.y_nm);
    }

    /// A circle's radius survives rotation via the circumference-point
    /// encoding.
    #[test]
    fn circle_center_rotates_and_radius_is_preserved() {
        let graphics = vec![IpcGraphicDefinition::Circle {
            center: (1.0, 0.0),
            end: (1.5, 0.0),
            layer: "F.Fab".to_string(),
            width: 0.1,
            filled: true,
        }];
        let fp = build(&graphics, 10.0, 10.0, 90.0);
        let shapes = shapes(&fp);
        let geometry = shapes[0].shape.as_ref().unwrap().geometry.as_ref().unwrap();
        let kiapi::common::types::graphic_shape::Geometry::Circle(c) = geometry else {
            panic!("expected Circle geometry, got {geometry:?}");
        };
        // Center (1,0) at 90° around (10,10): (10, 9).
        assert_eq!(c.center.unwrap().x_nm, 10_000_000);
        assert_eq!(c.center.unwrap().y_nm, 9_000_000);
        // Radius 0.5 mm regardless of rotation.
        assert_eq!(
            c.radius_point.unwrap().x_nm - c.center.unwrap().x_nm,
            500_000
        );
    }

    /// #117 guard for the pad path: the same PST_NORMAL rule that broke
    /// `add_via` applies to every pad we build, including the back-side case
    /// that B.Cu placement (#115) will eventually exercise.
    #[test]
    fn pad_stacks_are_unpackable_on_both_sides() {
        use crate::builders::tests::assert_normal_padstack_is_unpackable;

        let pad = |layer: &str| IpcPadDefinition {
            number: "1".to_string(),
            pad_type: "smd".to_string(),
            shape: "rect".to_string(),
            x: 0.0,
            y: 0.0,
            size_x: 1.0,
            size_y: 1.0,
            layers: vec![layer.to_string()],
            drill_x: None,
            drill_y: None,
            drill_oval: false,
            roundrect_ratio: 0.0,
            rotation: 0.0,
        };

        for layer in ["F.Cu", "B.Cu"] {
            let client = KiCadIpcClient::new("tcp://never-dialed");
            let any = client
                .build_footprint_item(
                    "Lib:Fp",
                    "R1",
                    "R",
                    &[pad(layer)],
                    &[],
                    &crate::types::IpcFieldPlacement::default(),
                    10.0,
                    10.0,
                    0.0,
                    "F.Cu",
                )
                .unwrap();
            let fp = kiapi::board::types::FootprintInstance::decode(any.value.as_slice()).unwrap();
            let pad_any = fp
                .definition
                .as_ref()
                .unwrap()
                .items
                .iter()
                .find(|any| any.type_url.ends_with("types.Pad"))
                .expect("pad item");
            let decoded = kiapi::board::types::Pad::decode(pad_any.value.as_slice()).unwrap();
            assert_normal_padstack_is_unpackable(
                decoded.pad_stack.as_ref().expect("pad_stack"),
                &format!("build_footprint_item pad on {layer}"),
            );
        }
    }
}
