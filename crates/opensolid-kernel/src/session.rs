//! Modeling session: model registry, operation journal, undo/redo.
//!
//! Implements the copy-on-write snapshot architecture of
//! `spec/09-session.md`: before every successful mutating operation the
//! session freezes its state via [`Arena::snapshot`] (chunk-pointer clones,
//! no per-entity copies) and pushes it on the undo stack. Undo/redo swap
//! whole states, so no operation ever implements its own inverse — every
//! future operation gets undo for free.
//!
//! The journal is an append-only audit trail of everything that happened,
//! including undos and redos; it is never rolled back. Named checkpoints
//! are user-visible save points, also outside undo's reach; restoring one
//! is itself an undoable operation.
//!
//! [`Session`] is generic over the model payload: the kernel registers
//! F-Rep [`Shape`]s today ([`Model`]) and can grow B-Rep model kinds
//! without touching the session machinery.

use opensolid_core::arena::{Arena, ArenaSnapshot, EntityId};
use opensolid_frep::Shape;
use std::collections::HashMap;
use thiserror::Error;

/// A registered F-Rep model: the payload of the default kernel session.
#[derive(Clone)]
pub struct Model {
    pub name: String,
    pub shape: Shape,
}

/// One entry in the session's append-only operation journal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct JournalEntry {
    /// Position in the journal, starting at 0.
    pub seq: usize,
    /// Human-readable description of the operation.
    pub description: String,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum SessionError {
    /// The id does not refer to a live model (removed, undone, or foreign).
    #[error("stale or unknown model id")]
    StaleId,
    #[error("nothing to undo")]
    NothingToUndo,
    #[error("nothing to redo")]
    NothingToRedo,
    #[error("no checkpoint named {0:?}")]
    UnknownCheckpoint(String),
}

/// Everything undo must restore: the registry and its insertion order.
struct State<M> {
    models: ArenaSnapshot<M>,
    order: Vec<EntityId<M>>,
}

/// A modeling session owning a registry of models, with journaled
/// operations, undo/redo, and named checkpoints. See the module docs.
pub struct Session<M: Clone = Model> {
    models: Arena<M>,
    /// Live ids in insertion order (what [`Session::iter`] walks).
    order: Vec<EntityId<M>>,
    journal: Vec<JournalEntry>,
    undo_stack: Vec<State<M>>,
    redo_stack: Vec<State<M>>,
    checkpoints: HashMap<String, State<M>>,
}

impl<M: Clone> Session<M> {
    pub fn new() -> Self {
        Self {
            models: Arena::new(),
            order: Vec::new(),
            journal: Vec::new(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            checkpoints: HashMap::new(),
        }
    }

    // --- registry (read) ---

    pub fn get(&self, id: EntityId<M>) -> Option<&M> {
        self.models.get(id)
    }

    /// Live models in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (EntityId<M>, &M)> {
        self.order
            .iter()
            .map(|&id| (id, self.models.get(id).expect("order tracks live ids")))
    }

    pub fn len(&self) -> usize {
        self.order.len()
    }

    pub fn is_empty(&self) -> bool {
        self.order.is_empty()
    }

    // --- journal ---

    /// The append-only audit trail, in execution order.
    pub fn journal(&self) -> &[JournalEntry] {
        &self.journal
    }

    fn record(&mut self, description: String) {
        self.journal.push(JournalEntry {
            seq: self.journal.len(),
            description,
        });
    }

    // --- undo/redo plumbing ---

    fn capture(&self) -> State<M> {
        State {
            models: self.models.snapshot(),
            order: self.order.clone(),
        }
    }

    fn apply(&mut self, state: &State<M>) {
        self.models.restore(&state.models);
        self.order = state.order.clone();
    }

    /// Freeze the pre-op state as an undo point. Any op that reaches this
    /// invalidates the redo timeline.
    fn mark_undo_point(&mut self) {
        self.undo_stack.push(self.capture());
        self.redo_stack.clear();
    }

    pub fn can_undo(&self) -> bool {
        !self.undo_stack.is_empty()
    }

    pub fn can_redo(&self) -> bool {
        !self.redo_stack.is_empty()
    }

    /// Roll back to the state before the most recent operation.
    pub fn undo(&mut self) -> Result<(), SessionError> {
        let state = self.undo_stack.pop().ok_or(SessionError::NothingToUndo)?;
        self.redo_stack.push(self.capture());
        self.apply(&state);
        self.record("undo".into());
        Ok(())
    }

    /// Replay the most recently undone operation.
    pub fn redo(&mut self) -> Result<(), SessionError> {
        let state = self.redo_stack.pop().ok_or(SessionError::NothingToRedo)?;
        self.undo_stack.push(self.capture());
        self.apply(&state);
        self.record("redo".into());
        Ok(())
    }

    // --- mutating operations (each: undo point + journal entry) ---

    /// Register a model; returns its id.
    pub fn create(&mut self, model: M) -> EntityId<M> {
        self.mark_undo_point();
        let id = self.models.insert(model);
        self.order.push(id);
        self.record(format!("create {id:?}"));
        id
    }

    /// Mutate a model in place. No undo point or journal entry is created
    /// if the id is stale — failed operations leave no trace.
    pub fn modify(&mut self, id: EntityId<M>, f: impl FnOnce(&mut M)) -> Result<(), SessionError> {
        if self.models.get(id).is_none() {
            return Err(SessionError::StaleId);
        }
        self.mark_undo_point();
        f(self.models.get_mut(id).expect("checked live above"));
        self.record(format!("modify {id:?}"));
        Ok(())
    }

    /// Remove a model from the registry.
    pub fn remove(&mut self, id: EntityId<M>) -> Result<M, SessionError> {
        if self.models.get(id).is_none() {
            return Err(SessionError::StaleId);
        }
        self.mark_undo_point();
        let model = self.models.remove(id).expect("checked live above");
        self.order.retain(|&o| o != id);
        self.record(format!("remove {id:?}"));
        Ok(model)
    }

    // --- named checkpoints ---

    /// Save the current state under `name`, replacing any previous
    /// checkpoint with that name. Cheap (copy-on-write) and outside the
    /// undo timeline: taking a checkpoint does not disturb redo.
    pub fn checkpoint(&mut self, name: &str) {
        self.checkpoints.insert(name.to_string(), self.capture());
        self.record(format!("checkpoint {name:?}"));
    }

    /// Restore a named checkpoint. This is a mutating operation: it creates
    /// an undo point, so it can itself be undone.
    pub fn restore(&mut self, name: &str) -> Result<(), SessionError> {
        let state = self
            .checkpoints
            .remove(name)
            .ok_or_else(|| SessionError::UnknownCheckpoint(name.to_string()))?;
        self.mark_undo_point();
        self.apply(&state);
        self.checkpoints.insert(name.to_string(), state);
        self.record(format!("restore {name:?}"));
        Ok(())
    }

    /// Names of all saved checkpoints (unordered).
    pub fn checkpoint_names(&self) -> impl Iterator<Item = &str> {
        self.checkpoints.keys().map(String::as_str)
    }
}

impl<M: Clone> Default for Session<M> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::types::Point3;
    use opensolid_frep::primitives::Sphere;

    fn sphere_shape(radius: f64) -> Shape {
        Shape::new(Sphere {
            center: Point3::origin(),
            radius,
        })
    }

    fn model(name: &str, radius: f64) -> Model {
        Model {
            name: name.into(),
            shape: sphere_shape(radius),
        }
    }

    /// Assert two sessions' registries are the same models in the same
    /// order, down to shape instance identity.
    fn assert_state(session: &Session<Model>, expected: &[(EntityId<Model>, &str, &Shape)]) {
        assert_eq!(session.len(), expected.len());
        for ((id, m), &(want_id, want_name, want_shape)) in session.iter().zip(expected) {
            assert_eq!(id, want_id);
            assert_eq!(m.name, want_name);
            assert!(
                m.shape.ptr_eq(want_shape),
                "shape of {want_name} is not the original instance"
            );
        }
    }

    #[test]
    fn create_modify_undo_restores_exact_prior_state() {
        let mut s: Session<Model> = Session::new();
        let original = sphere_shape(1.0);
        let a = s.create(Model {
            name: "a".into(),
            shape: original.clone(),
        });
        let b = s.create(model("b", 2.0));
        let b_shape = s.get(b).unwrap().shape.clone();

        let replacement = sphere_shape(3.0);
        s.modify(a, |m| {
            m.name = "a-edited".into();
            m.shape = replacement.clone();
        })
        .unwrap();
        assert_eq!(s.get(a).unwrap().name, "a-edited");
        assert!(s.get(a).unwrap().shape.ptr_eq(&replacement));

        s.undo().unwrap();
        // Exact prior state: same ids, names, and shape instances.
        assert_state(&s, &[(a, "a", &original), (b, "b", &b_shape)]);
    }

    #[test]
    fn undo_of_create_and_remove_round_trips() {
        let mut s: Session<Model> = Session::new();
        let a = s.create(model("a", 1.0));
        let b = s.create(model("b", 2.0));

        s.remove(a).unwrap();
        assert!(s.get(a).is_none());
        assert_eq!(s.len(), 1);

        s.undo().unwrap(); // un-remove: a is live again under the same id
        assert_eq!(s.get(a).unwrap().name, "a");
        assert_eq!(s.iter().map(|(id, _)| id).collect::<Vec<_>>(), [a, b]);

        s.undo().unwrap(); // un-create b
        assert!(s.get(b).is_none());
        assert_eq!(s.len(), 1);

        s.undo().unwrap(); // un-create a
        assert!(s.is_empty());
        assert_eq!(s.undo(), Err(SessionError::NothingToUndo));
    }

    #[test]
    fn redo_replays_undone_operations() {
        let mut s: Session<Model> = Session::new();
        let a = s.create(model("a", 1.0));
        let replacement = sphere_shape(9.0);
        s.modify(a, |m| m.shape = replacement.clone()).unwrap();

        s.undo().unwrap();
        s.undo().unwrap();
        assert!(s.is_empty());

        s.redo().unwrap(); // create a
        assert_eq!(s.get(a).unwrap().name, "a");
        s.redo().unwrap(); // modify a
        assert!(s.get(a).unwrap().shape.ptr_eq(&replacement));
        assert_eq!(s.redo(), Err(SessionError::NothingToRedo));
    }

    #[test]
    fn new_operation_invalidates_redo() {
        let mut s: Session<Model> = Session::new();
        s.create(model("a", 1.0));
        s.undo().unwrap();
        assert!(s.can_redo());
        s.create(model("b", 2.0));
        assert!(!s.can_redo());
        assert_eq!(s.redo(), Err(SessionError::NothingToRedo));
    }

    #[test]
    fn journal_lists_operations_in_order() {
        let mut s: Session<Model> = Session::new();
        let a = s.create(model("a", 1.0));
        s.modify(a, |m| m.name = "a2".into()).unwrap();
        s.checkpoint("cp");
        s.remove(a).unwrap();
        s.undo().unwrap();
        s.redo().unwrap();
        s.restore("cp").unwrap();

        let journal = s.journal();
        assert_eq!(journal.len(), 7);
        for (i, entry) in journal.iter().enumerate() {
            assert_eq!(entry.seq, i);
        }
        let descriptions: Vec<&str> = journal.iter().map(|e| e.description.as_str()).collect();
        assert!(descriptions[0].starts_with("create"));
        assert!(descriptions[1].starts_with("modify"));
        assert_eq!(descriptions[2], "checkpoint \"cp\"");
        assert!(descriptions[3].starts_with("remove"));
        assert_eq!(descriptions[4], "undo");
        assert_eq!(descriptions[5], "redo");
        assert_eq!(descriptions[6], "restore \"cp\"");
    }

    #[test]
    fn failed_operations_leave_no_trace() {
        let mut s: Session<Model> = Session::new();
        let a = s.create(model("a", 1.0));
        s.remove(a).unwrap();
        let journal_len = s.journal().len();
        let undo_depth = s.undo_stack.len();

        assert_eq!(s.modify(a, |_| {}), Err(SessionError::StaleId));
        assert!(matches!(s.remove(a), Err(SessionError::StaleId)));
        assert_eq!(
            s.restore("nope"),
            Err(SessionError::UnknownCheckpoint("nope".into()))
        );
        assert_eq!(s.journal().len(), journal_len);
        assert_eq!(s.undo_stack.len(), undo_depth);
    }

    #[test]
    fn named_checkpoint_restores_and_is_undoable() {
        let mut s: Session<Model> = Session::new();
        let original = sphere_shape(1.0);
        let a = s.create(Model {
            name: "a".into(),
            shape: original.clone(),
        });
        s.checkpoint("before-edits");

        let replacement = sphere_shape(2.0);
        s.modify(a, |m| {
            m.name = "a-edited".into();
            m.shape = replacement.clone();
        })
        .unwrap();
        let b = s.create(model("b", 3.0));

        s.restore("before-edits").unwrap();
        assert_state(&s, &[(a, "a", &original)]);
        assert!(s.get(b).is_none());
        // Restoring twice works (checkpoints are not consumed).
        s.restore("before-edits").unwrap();

        // The restore itself is one undo step.
        s.undo().unwrap();
        s.undo().unwrap();
        assert_eq!(s.get(a).unwrap().name, "a-edited");
        assert!(s.get(b).is_some());
        assert_eq!(s.checkpoint_names().collect::<Vec<_>>(), ["before-edits"]);
    }

    /// A payload whose clones are observable, to prove snapshots are
    /// copy-on-write rather than deep copies.
    #[derive(Debug)]
    struct Probe {
        clones: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    }

    impl Clone for Probe {
        fn clone(&self) -> Self {
            self.clones
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            Probe {
                clones: self.clones.clone(),
            }
        }
    }

    #[test]
    fn snapshots_do_not_deep_copy_the_registry() {
        let mut s: Session<Probe> = Session::new();
        let counter = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
        // Fill several arena chunks; every create snapshots the arena.
        let ids: Vec<_> = (0..200)
            .map(|_| {
                s.create(Probe {
                    clones: counter.clone(),
                })
            })
            .collect();
        counter.store(0, std::sync::atomic::Ordering::Relaxed);

        // 50 modifications of one model near the end of the arena, each
        // taking an undo snapshot of all 200 models.
        for _ in 0..50 {
            s.modify(ids[199], |_| {}).unwrap();
        }

        // Copy-on-write may re-clone the touched chunk (up to 64 entries per
        // op) but must never deep-copy the whole registry per snapshot.
        let clones = counter.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            clones <= 50 * 8,
            "expected chunk-local cloning, saw {clones} clones \
             (a deep copy would be 200 per op = 10000)"
        );
        assert!(s.can_undo());
    }
}
