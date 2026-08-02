use opensolid_brep::{GeometryStore, TopologyStore, primitives};
use opensolid_kernel::io::step::{StepWriteOptions, write_step};

fn main() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::cylinder(&mut store, &mut geo, 5.0, 12.0).unwrap();
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap();
    println!("{text}");
}
