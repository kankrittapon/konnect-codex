pub mod meta_tools;
pub mod registry;

use crate::tools::ToolDef;
use std::collections::{HashMap, HashSet};
use tokio::sync::RwLock;

/// Tracks which toolsets are currently loaded in the session.
pub struct ToolRouter {
    /// All registered toolset definitions
    registry: &'static [ToolsetMeta],
    /// Names of currently active toolsets
    active: RwLock<HashSet<String>>,
    /// Flat map of tool_name → ToolDef for fast dispatch
    loaded_tools: RwLock<HashMap<String, ToolDef>>,
}

/// Static metadata for a toolset (not the tools themselves).
#[derive(Debug, Clone)]
pub struct ToolsetMeta {
    pub name: &'static str,
    pub description: &'static str,
    pub category: &'static str,
    pub tool_count: usize,
}

impl ToolRouter {
    pub fn new() -> Self {
        ToolRouter {
            registry: registry::ALL_TOOLSETS,
            active: RwLock::new(HashSet::new()),
            loaded_tools: RwLock::new(HashMap::new()),
        }
    }

    pub fn all_toolsets(&self) -> &'static [ToolsetMeta] {
        self.registry
    }

    pub async fn load(&self, name: &str) -> Option<Vec<ToolDef>> {
        let defs = registry::tools_for(name)?;
        let mut active = self.active.write().await;
        let mut loaded = self.loaded_tools.write().await;
        active.insert(name.to_string());
        for def in &defs {
            loaded.insert(def.name.to_string(), def.clone());
        }
        Some(defs)
    }

    /// Load the starter kit — a minimal set of toolsets that every session needs.
    ///
    /// This is what runs at server startup. Additional toolsets are loaded on demand
    /// by the LLM calling `load_toolset(name)`. Keeping the baseline small means
    /// `tools/list` costs ~2K tokens instead of ~23K.
    pub async fn load_starter_kit(&self) {
        for name in registry::STARTER_KIT {
            let _ = self.load(name).await;
        }
    }

    /// Find which toolset a tool name belongs to, whether or not that toolset
    /// is currently loaded. Used to give the LLM an actionable error when it
    /// calls a tool whose toolset hasn't been loaded yet.
    pub fn find_toolset_for_tool(&self, tool_name: &str) -> Option<&'static str> {
        for ts in self.registry {
            if let Some(defs) = registry::tools_for(ts.name) {
                if defs.iter().any(|d| d.name == tool_name) {
                    return Some(ts.name);
                }
            }
        }
        None
    }

    pub async fn unload(&self, name: &str) -> bool {
        let defs = match registry::tools_for(name) {
            Some(d) => d,
            None => return false,
        };
        let mut active = self.active.write().await;
        let mut loaded = self.loaded_tools.write().await;
        active.remove(name);
        for def in &defs {
            loaded.remove(def.name);
        }
        true
    }

    pub async fn active_names(&self) -> Vec<String> {
        self.active.read().await.iter().cloned().collect()
    }

    pub async fn get_tool(&self, name: &str) -> Option<ToolDef> {
        self.loaded_tools.read().await.get(name).cloned()
    }

    /// Return all currently active ToolDefs for use in MCP tool listings.
    pub async fn active_tools(&self) -> Vec<ToolDef> {
        self.loaded_tools.read().await.values().cloned().collect()
    }
}

impl Default for ToolRouter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn every_toolset_loads() {
        let router = ToolRouter::new();
        for meta in registry::ALL_TOOLSETS {
            assert!(
                router.load(meta.name).await.is_some(),
                "toolset '{}' failed to load",
                meta.name
            );
        }
        assert!(router.load("nonexistent_toolset").await.is_none());
    }

    #[tokio::test]
    async fn starter_kit_loads_expected_toolsets_and_nothing_more() {
        let router = ToolRouter::new();
        router.load_starter_kit().await;
        let active: std::collections::HashSet<String> =
            router.active_names().await.into_iter().collect();
        let expected: std::collections::HashSet<String> = [
            "config",
            "project",
            "pcb_board",
            "pcb_components",
            "pcb_routing",
            "pcb_export",
            "verification",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect();
        assert_eq!(active, expected);
        for expected in registry::STARTER_KIT {
            assert!(
                active.contains(*expected),
                "starter kit missing toolset '{}'",
                expected
            );
        }
        assert_eq!(active.len(), registry::STARTER_KIT.len());
        // On-demand toolsets must not be auto-loaded
        assert!(!active.contains("integration"));
        assert!(!active.contains("templates"));
    }

    #[tokio::test]
    async fn starter_kit_exposes_refill_and_routing_inspection_without_dynamic_load() {
        let router = ToolRouter::new();
        router.load_starter_kit().await;

        assert!(router.get_tool("refill_zones").await.is_some());
        assert!(router.get_tool("get_routing_geometry").await.is_some());
        assert!(router.get_tool("check_route_clearance").await.is_some());
        assert!(router.get_tool("check_via_clearance").await.is_some());
    }

    #[tokio::test]
    async fn find_toolset_for_tool_resolves_unloaded_tools() {
        let router = ToolRouter::new();
        router.load_starter_kit().await;
        // Lookup remains available for tools in an already-loaded PCB toolset.
        assert_eq!(
            router.find_toolset_for_tool("place_component"),
            Some("pcb_components")
        );
        assert_eq!(
            router.find_toolset_for_tool("route_trace"),
            Some("pcb_routing")
        );
        assert_eq!(router.find_toolset_for_tool("nonexistent_tool"), None);
    }

    // ─── Registry invariants ─────────────────────────────────────────────────
    //
    // These are the guardrails that protect future work:
    //
    // - The hand-written `tool_count` in ALL_TOOLSETS must match what
    //   `tools_for(name)` actually returns. Otherwise `list_toolboxes` lies
    //   to the LLM.
    // - No toolset grows past ~20 tools. Past that, split it — 20 tool
    //   descriptions at ~400 bytes each is already a 1.6KB payload in
    //   `tools/list` when loaded, and tool selection accuracy degrades.

    /// The cap above which a toolset must be split. If you hit this, either
    /// move tools to a sibling toolset or add a new one — don't raise this
    /// number without a conversation.
    const MAX_TOOLS_PER_TOOLSET: usize = 20;

    #[test]
    fn registry_tool_counts_match_reality() {
        for meta in registry::ALL_TOOLSETS {
            let defs = registry::tools_for(meta.name)
                .unwrap_or_else(|| panic!("tools_for({}) returned None", meta.name));
            assert_eq!(
                defs.len(),
                meta.tool_count,
                "registry declares tool_count={} for '{}' but tools_for() returned {} tools — \
                 update ALL_TOOLSETS in router/registry.rs",
                meta.tool_count,
                meta.name,
                defs.len()
            );
        }
    }

    #[test]
    fn pcb_export_registry_count_matches_implementation() {
        let meta = registry::ALL_TOOLSETS
            .iter()
            .find(|meta| meta.name == "pcb_export")
            .expect("pcb_export metadata missing");
        assert_eq!(meta.tool_count, crate::tools::pcb_export::tools().len());
        assert_eq!(meta.tool_count, 13);
    }

    #[test]
    fn no_toolset_has_duplicate_tool_names() {
        for meta in registry::ALL_TOOLSETS {
            let defs = registry::tools_for(meta.name).unwrap();
            let mut seen = std::collections::HashSet::new();
            for d in &defs {
                assert!(
                    seen.insert(d.name),
                    "duplicate tool name '{}' inside toolset '{}'",
                    d.name,
                    meta.name
                );
            }
        }
    }

    #[test]
    fn tool_names_unique_across_toolsets() {
        // Duplicate names across toolsets are a silent foot-gun: whichever
        // toolset is loaded last wins in `loaded_tools`, so behavior depends
        // on load order. Aliases that point at the same handler are fine; the
        // test fails on first occurrence so the committer has to decide.
        let mut owner: std::collections::HashMap<&'static str, &'static str> =
            std::collections::HashMap::new();
        let mut collisions = Vec::new();
        for meta in registry::ALL_TOOLSETS {
            let defs = registry::tools_for(meta.name).unwrap();
            for d in &defs {
                if let Some(prev) = owner.insert(d.name, meta.name) {
                    if prev != meta.name {
                        collisions.push(format!(
                            "'{}' declared in both '{}' and '{}'",
                            d.name, prev, meta.name
                        ));
                    }
                }
            }
        }
        assert!(
            collisions.is_empty(),
            "tool name collisions across toolsets (last-loaded wins in the router):\n  {}",
            collisions.join("\n  ")
        );
    }

    #[test]
    fn no_toolset_exceeds_max_size() {
        for meta in registry::ALL_TOOLSETS {
            assert!(
                meta.tool_count <= MAX_TOOLS_PER_TOOLSET,
                "toolset '{}' has {} tools, which exceeds the soft cap of {}. \
                 Split it into two before bumping this cap.",
                meta.name,
                meta.tool_count,
                MAX_TOOLS_PER_TOOLSET
            );
        }
    }

    #[test]
    fn starter_kit_entries_are_all_valid_toolsets() {
        for name in registry::STARTER_KIT {
            assert!(
                registry::tools_for(name).is_some(),
                "STARTER_KIT references unknown toolset '{}'",
                name
            );
        }
    }
}
