//! Generational arena with copy-on-write snapshots.
//!
//! Entities live in fixed-size chunks, each behind an [`Arc`]. Taking an
//! [`ArenaSnapshot`] clones only the chunk pointers and the free list, so it
//! costs O(n / CHUNK_SIZE) regardless of entity size. A mutation touching a
//! chunk that a snapshot still shares clones just that chunk
//! ([`Arc::make_mut`]); chunks owned exclusively by the arena mutate in
//! place. Snapshots are therefore fully isolated from later mutations, which
//! is the primitive the session layer builds undo/redo on
//! (`spec/09-session.md`).
//!
//! Stale-ID safety: every insert stamps its slot with a fresh value from a
//! per-arena monotonic counter. The counter is deliberately *not* rolled
//! back by [`Arena::restore`], so an ID minted after a snapshot can never
//! alias an entity created after restoring that snapshot (the classic ABA
//! hazard of rollback systems).

use std::marker::PhantomData;
use std::sync::Arc;

/// Entities per copy-on-write chunk. Larger chunks make snapshots cheaper
/// but increase the clone cost of the first post-snapshot write to a chunk.
const CHUNK_SIZE: usize = 64;

pub struct EntityId<T> {
    pub(crate) index: u32,
    pub(crate) generation: u32,
    _phantom: PhantomData<T>,
}

// Manual impls: deriving would bound them on `T: Copy` etc. via PhantomData,
// but an id is copyable/comparable regardless of the entity type it names.
impl<T> Clone for EntityId<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> Copy for EntityId<T> {}

impl<T> PartialEq for EntityId<T> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index && self.generation == other.generation
    }
}

impl<T> Eq for EntityId<T> {}

impl<T> std::hash::Hash for EntityId<T> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.index.hash(state);
        self.generation.hash(state);
    }
}

impl<T> std::fmt::Debug for EntityId<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "EntityId<{}>({}, gen {})",
            std::any::type_name::<T>()
                .rsplit("::")
                .next()
                .unwrap_or("?"),
            self.index,
            self.generation
        )
    }
}

impl<T> EntityId<T> {
    fn new(index: u32, generation: u32) -> Self {
        Self {
            index,
            generation,
            _phantom: PhantomData,
        }
    }
}

#[derive(Clone)]
struct ArenaEntry<T> {
    value: Option<T>,
    generation: u32,
}

type Chunk<T> = Arc<Vec<ArenaEntry<T>>>;

pub struct Arena<T> {
    chunks: Vec<Chunk<T>>,
    free_list: Vec<u32>,
    len: u32,
    /// Monotonic source of entry generations. Never decreases, not even on
    /// [`Arena::restore`] — see the module docs on ABA safety.
    next_generation: u32,
}

/// A frozen, read-only view of an [`Arena`] at the moment
/// [`Arena::snapshot`] was called. Shares storage with the arena until the
/// arena mutates (copy-on-write), so it is cheap to take and hold.
#[derive(Clone)]
pub struct ArenaSnapshot<T> {
    chunks: Vec<Chunk<T>>,
    free_list: Vec<u32>,
    len: u32,
}

impl<T> Arena<T> {
    pub fn new() -> Self {
        Self {
            chunks: Vec::new(),
            free_list: Vec::new(),
            len: 0,
            next_generation: 0,
        }
    }

    fn entry(&self, index: u32) -> Option<&ArenaEntry<T>> {
        let i = index as usize;
        self.chunks.get(i / CHUNK_SIZE)?.get(i % CHUNK_SIZE)
    }

    /// Total slots ever allocated (live + freed).
    fn slot_count(&self) -> u32 {
        match self.chunks.last() {
            None => 0,
            Some(last) => ((self.chunks.len() - 1) * CHUNK_SIZE + last.len()) as u32,
        }
    }

    fn fresh_generation(&mut self) -> u32 {
        let g = self.next_generation;
        self.next_generation = g.checked_add(1).expect("arena generation counter overflow");
        g
    }

    pub fn get(&self, id: EntityId<T>) -> Option<&T> {
        let entry = self.entry(id.index)?;
        if entry.generation == id.generation {
            entry.value.as_ref()
        } else {
            None
        }
    }

    pub fn len(&self) -> u32 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Freeze the current state. O(chunk count), independent of entity size;
    /// later mutations of the arena never affect the snapshot.
    pub fn snapshot(&self) -> ArenaSnapshot<T> {
        ArenaSnapshot {
            chunks: self.chunks.clone(),
            free_list: self.free_list.clone(),
            len: self.len,
        }
    }

    /// Roll the arena back to a previously taken snapshot. IDs minted before
    /// the snapshot become valid again; IDs minted after it turn stale and
    /// stay stale forever (the generation counter is not rolled back).
    pub fn restore(&mut self, snapshot: &ArenaSnapshot<T>) {
        self.chunks = snapshot.chunks.clone();
        self.free_list = snapshot.free_list.clone();
        self.len = snapshot.len;
    }

    /// Iterate over every live entity as `(id, &value)`, in slot order.
    /// Freed slots are skipped; the yielded ids resolve via [`Arena::get`].
    pub fn iter(&self) -> impl Iterator<Item = (EntityId<T>, &T)> {
        self.chunks.iter().enumerate().flat_map(|(ci, chunk)| {
            chunk.iter().enumerate().filter_map(move |(i, entry)| {
                let value = entry.value.as_ref()?;
                let index = (ci * CHUNK_SIZE + i) as u32;
                Some((EntityId::new(index, entry.generation), value))
            })
        })
    }
}

impl<T: Clone> Arena<T> {
    /// Copy-on-write access to a slot: clones the containing chunk first if
    /// a snapshot still shares it.
    fn entry_mut(&mut self, index: u32) -> Option<&mut ArenaEntry<T>> {
        let i = index as usize;
        let chunk = self.chunks.get_mut(i / CHUNK_SIZE)?;
        Arc::make_mut(chunk).get_mut(i % CHUNK_SIZE)
    }

    pub fn insert(&mut self, value: T) -> EntityId<T> {
        self.len += 1;
        let generation = self.fresh_generation();
        if let Some(index) = self.free_list.pop() {
            let entry = self.entry_mut(index).expect("free-list index in bounds");
            entry.generation = generation;
            entry.value = Some(value);
            EntityId::new(index, generation)
        } else {
            let index = self.slot_count();
            if self.chunks.last().is_none_or(|c| c.len() == CHUNK_SIZE) {
                self.chunks.push(Arc::new(Vec::with_capacity(CHUNK_SIZE)));
            }
            let chunk = self.chunks.last_mut().expect("just ensured a chunk");
            Arc::make_mut(chunk).push(ArenaEntry {
                value: Some(value),
                generation,
            });
            EntityId::new(index, generation)
        }
    }

    pub fn get_mut(&mut self, id: EntityId<T>) -> Option<&mut T> {
        let entry = self.entry_mut(id.index)?;
        if entry.generation == id.generation {
            entry.value.as_mut()
        } else {
            None
        }
    }

    pub fn remove(&mut self, id: EntityId<T>) -> Option<T> {
        let entry = self.entry_mut(id.index)?;
        if entry.generation == id.generation {
            let value = entry.value.take()?;
            self.len -= 1;
            self.free_list.push(id.index);
            Some(value)
        } else {
            None
        }
    }
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> ArenaSnapshot<T> {
    /// Look up an entity as of the snapshot. IDs minted after the snapshot
    /// resolve to `None` even if they are live in the arena.
    pub fn get(&self, id: EntityId<T>) -> Option<&T> {
        let i = id.index as usize;
        let entry = self.chunks.get(i / CHUNK_SIZE)?.get(i % CHUNK_SIZE)?;
        if entry.generation == id.generation {
            entry.value.as_ref()
        } else {
            None
        }
    }

    pub fn len(&self) -> u32 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_get() {
        let mut arena: Arena<String> = Arena::new();
        let id = arena.insert("hello".into());
        assert_eq!(arena.get(id).unwrap(), "hello");
    }

    #[test]
    fn remove_invalidates_id() {
        let mut arena: Arena<i32> = Arena::new();
        let id = arena.insert(42);
        assert_eq!(arena.remove(id), Some(42));
        assert!(arena.get(id).is_none());
        // Double remove is a no-op, not a double free.
        assert_eq!(arena.remove(id), None);
        assert_eq!(arena.len(), 0);
    }

    #[test]
    fn iter_visits_live_entities_with_resolvable_ids() {
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        let b = arena.insert(2);
        let c = arena.insert(3);
        arena.remove(b);

        let seen: Vec<_> = arena.iter().map(|(id, &v)| (id, v)).collect();
        assert_eq!(seen, vec![(a, 1), (c, 3)]);
        for (id, &v) in arena.iter() {
            assert_eq!(arena.get(id), Some(&v));
        }

        // A reused slot must yield the new id, not the stale one.
        let d = arena.insert(4);
        assert!(arena.iter().any(|(id, &v)| id == d && v == 4));
        assert!(arena.iter().all(|(id, _)| id != b));
    }

    #[test]
    fn generation_prevents_use_after_free() {
        let mut arena: Arena<i32> = Arena::new();
        let id1 = arena.insert(1);
        arena.remove(id1);
        let id2 = arena.insert(2);
        // Slot is reused but the stale id must not resolve.
        assert_eq!(id1.index, id2.index);
        assert!(arena.get(id1).is_none());
        assert!(arena.get_mut(id1).is_none());
        assert_eq!(arena.remove(id1), None);
        assert_eq!(arena.get(id2).unwrap(), &2);
    }

    #[test]
    fn ids_are_typed_and_hashable() {
        use std::collections::HashSet;
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        let b = arena.insert(2);
        let set: HashSet<_> = [a, b].into_iter().collect();
        assert_eq!(set.len(), 2);
        assert!(set.contains(&a));
    }

    #[test]
    fn grows_across_chunk_boundaries() {
        let mut arena: Arena<usize> = Arena::new();
        let ids: Vec<_> = (0..CHUNK_SIZE * 3 + 5).map(|i| arena.insert(i)).collect();
        assert_eq!(arena.len() as usize, CHUNK_SIZE * 3 + 5);
        for (i, id) in ids.iter().enumerate() {
            assert_eq!(arena.get(*id), Some(&i));
        }
    }

    #[test]
    fn snapshot_isolated_from_later_mutations() {
        let mut arena: Arena<String> = Arena::new();
        let kept = arena.insert("kept".into());
        let edited = arena.insert("original".into());
        let removed = arena.insert("removed".into());

        let snap = arena.snapshot();

        *arena.get_mut(edited).unwrap() = "edited".into();
        arena.remove(removed);
        let added = arena.insert("added".into());

        // The arena sees the new state...
        assert_eq!(arena.get(edited).unwrap(), "edited");
        assert!(arena.get(removed).is_none());
        assert_eq!(arena.get(added).unwrap(), "added");
        // ...while the snapshot still sees the old one.
        assert_eq!(snap.get(kept).unwrap(), "kept");
        assert_eq!(snap.get(edited).unwrap(), "original");
        assert_eq!(snap.get(removed).unwrap(), "removed");
        assert!(snap.get(added).is_none());
        assert_eq!(snap.len(), 3);
    }

    #[test]
    fn snapshot_survives_growth_into_new_chunks() {
        let mut arena: Arena<usize> = Arena::new();
        let first = arena.insert(0);
        let snap = arena.snapshot();
        // Grow well past the chunk the snapshot shares.
        for i in 1..CHUNK_SIZE * 2 {
            arena.insert(i);
        }
        assert_eq!(snap.len(), 1);
        assert_eq!(snap.get(first), Some(&0));
    }

    #[test]
    fn restore_rolls_back_to_snapshot() {
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        let snap = arena.snapshot();

        let b = arena.insert(2);
        arena.remove(a);
        assert_eq!(arena.len(), 1);

        arena.restore(&snap);
        assert_eq!(arena.len(), 1);
        assert_eq!(arena.get(a), Some(&1));
        assert!(arena.get(b).is_none());
    }

    #[test]
    fn stale_id_from_discarded_timeline_never_aliases() {
        // The ABA hazard: mint an id after the snapshot, restore, then reuse
        // the same slot. The discarded-timeline id must stay stale.
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        let snap = arena.snapshot();

        arena.remove(a);
        let ghost = arena.insert(99); // reuses a's slot in the discarded timeline

        arena.restore(&snap);
        assert_eq!(arena.get(a), Some(&1));
        assert!(arena.get(ghost).is_none());

        // Reuse the slot again post-restore: ghost must still not resolve.
        arena.remove(a);
        let fresh = arena.insert(7);
        assert_eq!(ghost.index, fresh.index);
        assert!(arena.get(ghost).is_none());
        assert_eq!(arena.get(fresh), Some(&7));
    }

    #[test]
    fn multiple_snapshots_are_independent() {
        let mut arena: Arena<i32> = Arena::new();
        let a = arena.insert(1);
        let snap1 = arena.snapshot();
        let b = arena.insert(2);
        let snap2 = arena.snapshot();
        arena.remove(a);
        arena.remove(b);

        assert_eq!(snap1.len(), 1);
        assert_eq!(snap2.len(), 2);
        assert_eq!(snap1.get(a), Some(&1));
        assert!(snap1.get(b).is_none());
        assert_eq!(snap2.get(a), Some(&1));
        assert_eq!(snap2.get(b), Some(&2));

        // Restoring the older snapshot, then the newer, lands on each state.
        arena.restore(&snap1);
        assert_eq!(arena.len(), 1);
        arena.restore(&snap2);
        assert_eq!(arena.len(), 2);
        assert_eq!(arena.get(b), Some(&2));
    }

    #[test]
    fn reuse_after_free_prefers_freed_slots() {
        let mut arena: Arena<i32> = Arena::new();
        let ids: Vec<_> = (0..10).map(|i| arena.insert(i)).collect();
        arena.remove(ids[3]);
        arena.remove(ids[7]);
        let n = arena.insert(100);
        let m = arena.insert(200);
        // Freed slots are recycled (LIFO) before any new slot is allocated.
        assert_eq!(n.index, 7);
        assert_eq!(m.index, 3);
        assert_eq!(arena.len(), 10);
        assert_eq!(arena.slot_count(), 10);
    }
}
