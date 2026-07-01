# 09 — Session Management

Memory management, undo/redo, journaling, and state management.

## 1. Kernel Instance

The kernel is the central runtime that owns all entities and state:

```rust
pub struct Kernel {
    /// All topological entities.
    pub(crate) topology: TopologyStore,
    /// All geometric entities (curves, surfaces).
    pub(crate) geometry: GeometryStore,
    /// Tolerance configuration.
    pub(crate) config: ToleranceConfig,
    /// Undo/redo history.
    pub(crate) history: History,
    /// Entity attributes.
    pub(crate) attributes: AttributeStore,
}

impl Kernel {
    /// Create a new kernel instance with default configuration.
    pub fn new() -> Self;

    /// Create with custom tolerance configuration.
    pub fn with_config(config: ToleranceConfig) -> Self;

    /// Get kernel statistics.
    pub fn stats(&self) -> KernelStats;
}

pub struct KernelStats {
    pub bodies: usize,
    pub faces: usize,
    pub edges: usize,
    pub vertices: usize,
    pub curves: usize,
    pub surfaces: usize,
    pub memory_bytes: usize,
}
```

## 2. Undo/Redo System

### 2.1 Architecture: CoW Snapshots (Not Commands)

The kernel uses **copy-on-write arena snapshots** for undo/redo. This is the same
approach Parasolid uses (partition/rollback) and avoids the problems of the
Command pattern:

- Operations don't need to implement `undo()` (every new operation gets undo for free)
- No coupling between operation code and undo logic
- Snapshot granularity is independent of operation complexity
- Works for ALL operations without per-operation awareness

**Why not the Command pattern?** It requires every operation to produce a reversible
record. This is invasive — every future operation must cooperate. CoW snapshots
work at the arena level, below operations, requiring zero operation-level cooperation.

### 2.2 Snapshots (Copy-on-Write)

```rust
pub struct KernelSnapshot {
    topology: TopologySnapshot,
    geometry: GeometrySnapshot,
    attributes: AttributeSnapshot,
    description: String,
    timestamp: Instant,
}

/// Copy-on-write snapshot of an arena.
/// Only stores entities that changed since the snapshot was taken.
pub struct ArenaSnapshot<T> {
    /// The generation counter at snapshot time.
    generation: u64,
    /// Entities that were modified/deleted after this snapshot.
    overwritten: HashMap<u32, T>,
}

impl Kernel {
    /// Take a snapshot of current state (O(1) — just records generation).
    pub fn snapshot(&self, description: &str) -> SnapshotId;

    /// Restore a snapshot (replays overwrites in reverse).
    pub fn restore(&mut self, snapshot: SnapshotId) -> Result<(), KernelError>;

    /// Branch: create an independent copy of current state for exploration.
    /// AI agents use this to try multiple approaches.
    pub fn branch(&self) -> Kernel;
}
```

### 2.3 History Stack

```rust
pub struct History {
    snapshots: Vec<KernelSnapshot>,
    current: usize,
    /// Maximum history depth by count (0 = unlimited).
    max_depth: usize,
    /// Memory budget in bytes. When total snapshot memory exceeds this,
    /// oldest snapshots are evicted. Prevents unbounded memory growth
    /// during extended sessions (50+ operations).
    memory_budget_bytes: usize,
    /// Current memory usage of all retained snapshots.
    current_memory_bytes: usize,
}

impl History {
    pub fn undo(&mut self, kernel: &mut Kernel) -> Result<(), KernelError>;
    pub fn redo(&mut self, kernel: &mut Kernel) -> Result<(), KernelError>;
    pub fn can_undo(&self) -> bool;
    pub fn can_redo(&self) -> bool;
    pub fn clear(&mut self);
    pub fn history_descriptions(&self) -> Vec<&str>;
    /// Current memory used by undo history.
    pub fn memory_usage(&self) -> usize;
    /// Evict oldest snapshots until memory is within budget.
    fn evict_if_over_budget(&mut self);
}

impl Default for History {
    fn default() -> Self {
        Self {
            snapshots: Vec::new(),
            current: 0,
            max_depth: 100,
            memory_budget_bytes: 512 * 1024 * 1024,  // 512 MB default
            current_memory_bytes: 0,
        }
    }
}
```

CI benchmark: run 100 operations on a 100-face body, assert peak RSS < 1GB.
```

### 2.4 Automatic Snapshot Policy

The kernel automatically takes a snapshot before every top-level mutating operation.
"Top-level" means operations called from the public API — internal sub-operations
do NOT create individual snapshots.

```rust
impl Kernel {
    pub fn unite(&mut self, a: EntityId<Body>, b: EntityId<Body>) -> Result<EntityId<Body>, BooleanError> {
        // Snapshot is taken automatically by the operation dispatcher.
        // If the operation fails, the snapshot is discarded (no undo point for failures).
        // If it succeeds, the snapshot becomes an undo point.
        ...
    }
}
```

This means undo granularity matches the user's intent: each API call = one undo step.

## 3. Branching for AI Exploration

A key differentiator: AI agents can explore multiple design paths:

```rust
pub struct DesignBranch {
    pub id: BranchId,
    pub parent: Option<BranchId>,
    pub kernel: Kernel,
    pub description: String,
    pub created_at: Instant,
}

pub struct DesignTree {
    branches: HashMap<BranchId, DesignBranch>,
    current: BranchId,
}

impl DesignTree {
    /// Create a new branch from current state.
    pub fn branch(&mut self, description: &str) -> BranchId;

    /// Switch to a different branch.
    pub fn checkout(&mut self, branch: BranchId) -> Result<(), KernelError>;

    /// Merge a branch back (take its final state as current).
    pub fn merge(&mut self, branch: BranchId) -> Result<(), KernelError>;

    /// Compare two branches (what entities differ).
    pub fn diff(&self, a: BranchId, b: BranchId) -> BranchDiff;

    /// Discard a branch.
    pub fn discard(&mut self, branch: BranchId);
}
```

## 4. Journaling

For crash recovery and audit trails:

```rust
pub struct Journal {
    writer: BufWriter<File>,
    entries: usize,
}

pub struct JournalEntry {
    pub timestamp: u64,
    pub command_type: String,
    pub parameters: Vec<u8>,  // Serialized command parameters
    pub result: CommandResult,
}

impl Journal {
    /// Start journaling to a file.
    pub fn open(path: &Path) -> io::Result<Self>;

    /// Record a command execution.
    pub fn record(&mut self, entry: &JournalEntry) -> io::Result<()>;

    /// Replay a journal to reconstruct state.
    pub fn replay(path: &Path, kernel: &mut Kernel) -> Result<(), JournalError>;

    /// Flush to disk.
    pub fn flush(&mut self) -> io::Result<()>;
}
```

## 5. Native File Format

Fast binary serialization for kernel state (not for interchange — use STEP for that):

```rust
pub struct NativeFormat;

impl NativeFormat {
    /// Save kernel state to a binary file.
    /// Format is versioned for backward compatibility.
    pub fn save(kernel: &Kernel, path: &Path) -> io::Result<()>;

    /// Load kernel state from a binary file.
    pub fn load(path: &Path) -> Result<Kernel, NativeFormatError>;

    /// Save a single body (for partial save/load).
    pub fn save_body(
        body: EntityId<Body>,
        kernel: &Kernel,
        path: &Path,
    ) -> io::Result<()>;

    /// Load a body into an existing kernel.
    pub fn load_body(
        path: &Path,
        kernel: &mut Kernel,
    ) -> Result<EntityId<Body>, NativeFormatError>;
}
```

## 6. Garbage Collection

When entities are deleted (e.g., input bodies consumed by boolean), their storage
is not immediately freed. Periodic GC reclaims memory:

```rust
impl Kernel {
    /// Collect unreachable entities and free their storage.
    /// Only frees entities not reachable from any live body.
    pub fn gc(&mut self) -> GcStats;

    /// Compact arenas (eliminate gaps from deletions).
    /// Invalidates all existing EntityIds — use after bulk operations.
    pub fn compact(&mut self);
}

pub struct GcStats {
    pub freed_curves: usize,
    pub freed_surfaces: usize,
    pub freed_topology: usize,
    pub bytes_reclaimed: usize,
}
```

## 7. Thread Safety

```rust
// The kernel itself is NOT thread-safe (single-owner).
// For parallel read operations, use a shared reference:

impl Kernel {
    /// Get a read-only view for parallel queries.
    /// Multiple threads can query simultaneously.
    pub fn read(&self) -> KernelReadGuard<'_>;
}

pub struct KernelReadGuard<'a> {
    kernel: &'a Kernel,
}

impl<'a> KernelReadGuard<'a> {
    // All query operations available (mass props, tessellation, etc.)
    // No mutation allowed.
    pub fn tessellate_body(&self, body: EntityId<Body>, options: &TessellationOptions) -> BodyMesh;
    pub fn mass_properties(&self, body: EntityId<Body>) -> MassProperties;
    // ...
}
```
