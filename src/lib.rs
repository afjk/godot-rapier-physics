#![feature(try_blocks)]
#[cfg(feature = "scenesync-runtime")]
extern crate godot_scenesync as godot;
#[cfg(all(feature = "scenesync-runtime", any(feature = "dim2", feature = "dim3")))]
compile_error!("scenesync-runtime cannot be combined with the full PhysicsServer build");
#[cfg(all(feature = "single", feature = "dim2"))]
extern crate rapier2d as rapier;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate rapier2d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate rapier3d as rapier;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate rapier3d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim2"))]
extern crate salva2d as salva;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate salva2d_f64 as salva;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate salva3d as salva;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate salva3d_f64 as salva;
#[cfg(not(feature = "scenesync-runtime"))]
mod bodies;
#[cfg(not(feature = "scenesync-runtime"))]
mod fluids;
#[cfg(not(feature = "scenesync-runtime"))]
mod joints;
#[cfg(not(feature = "scenesync-runtime"))]
mod nodes;
#[cfg(not(feature = "scenesync-runtime"))]
mod rapier_wrapper;
#[cfg(any(
    feature = "scenesync-runtime",
    all(feature = "scenesync-parity", feature = "single", feature = "dim3")
))]
pub mod scenesync_parity;
#[cfg(not(feature = "scenesync-runtime"))]
mod servers;
#[cfg(not(feature = "scenesync-runtime"))]
mod shapes;
#[cfg(not(feature = "scenesync-runtime"))]
mod spaces;
#[cfg(not(feature = "scenesync-runtime"))]
mod types;
use godot::prelude::*;
#[cfg(feature = "dim2")]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 2D extension library.
pub struct RapierPhysics2DExtensionLibrary {}
#[cfg(feature = "dim2")]
#[gdextension(entry_symbol = rapier_2d_init)]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_stage_deinit(_level: InitStage) {}
}
#[cfg(any(feature = "dim3", feature = "scenesync-runtime"))]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 3D extension library.
pub struct RapierPhysics3DExtensionLibrary {}
#[cfg(any(feature = "dim3", feature = "scenesync-runtime"))]
#[gdextension(entry_symbol = rapier_3d_init)]
unsafe impl ExtensionLibrary for RapierPhysics3DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        #[cfg(not(feature = "scenesync-runtime"))]
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
        #[cfg(feature = "scenesync-runtime")]
        let _ = level;
    }

    fn on_stage_deinit(_level: InitStage) {}
}
