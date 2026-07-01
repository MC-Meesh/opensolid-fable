use std::marker::PhantomData;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EntityId<T> {
    pub(crate) index: u32,
    pub(crate) generation: u32,
    _phantom: PhantomData<T>,
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

struct ArenaEntry<T> {
    value: Option<T>,
    generation: u32,
}

pub struct Arena<T> {
    entries: Vec<ArenaEntry<T>>,
    free_list: Vec<u32>,
    len: u32,
}

impl<T> Arena<T> {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
            free_list: Vec::new(),
            len: 0,
        }
    }

    pub fn insert(&mut self, value: T) -> EntityId<T> {
        self.len += 1;
        if let Some(index) = self.free_list.pop() {
            let entry = &mut self.entries[index as usize];
            entry.generation += 1;
            entry.value = Some(value);
            EntityId::new(index, entry.generation)
        } else {
            let index = self.entries.len() as u32;
            self.entries.push(ArenaEntry {
                value: Some(value),
                generation: 0,
            });
            EntityId::new(index, 0)
        }
    }

    pub fn get(&self, id: EntityId<T>) -> Option<&T> {
        let entry = self.entries.get(id.index as usize)?;
        if entry.generation == id.generation {
            entry.value.as_ref()
        } else {
            None
        }
    }

    pub fn get_mut(&mut self, id: EntityId<T>) -> Option<&mut T> {
        let entry = self.entries.get_mut(id.index as usize)?;
        if entry.generation == id.generation {
            entry.value.as_mut()
        } else {
            None
        }
    }

    pub fn remove(&mut self, id: EntityId<T>) -> Option<T> {
        let entry = self.entries.get_mut(id.index as usize)?;
        if entry.generation == id.generation {
            self.len -= 1;
            self.free_list.push(id.index);
            entry.value.take()
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

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self::new()
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
        arena.remove(id);
        assert!(arena.get(id).is_none());
    }

    #[test]
    fn generation_prevents_use_after_free() {
        let mut arena: Arena<i32> = Arena::new();
        let id1 = arena.insert(1);
        arena.remove(id1);
        let id2 = arena.insert(2);
        assert!(arena.get(id1).is_none());
        assert_eq!(arena.get(id2).unwrap(), &2);
    }
}
