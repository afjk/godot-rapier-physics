//! SceneSync's deterministic Rapier 3D profile.
//!
//! This module deliberately uses an upstream Rapier 0.30.0 dependency that is
//! separate from the patched Rapier revision backing Godot's PhysicsServer.
use godot::prelude::*;
use scenesync_rapier3d::na::Isometry3;
use scenesync_rapier3d::na::Quaternion as NaQuaternion;
use scenesync_rapier3d::na::Translation3;
use scenesync_rapier3d::na::UnitQuaternion;
use scenesync_rapier3d::prelude::*;
pub const PROFILE: &str = "SceneSyncRapierParity-0.30";
pub const RAPIER_CORE_VERSION: &str = "0.30.0";
pub const HASH_VERSION: &str = "SceneSyncCanonicalPhysicsHashV1";
const FNV64_OFFSET: u64 = 0xcbf29ce484222325;
const FNV64_PRIME: u64 = 0x100000001b3;
const INITIAL_NEXT_PID_CONTROLLER_ID: u64 = 1;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeKind {
    Sphere,
    Box,
}
#[derive(Clone, Debug)]
pub struct BodyDefinition {
    pub id: String,
    pub fixed: bool,
    pub shape: ShapeKind,
    pub position: [f32; 3],
    /// Quaternion components in SceneSync order: x, y, z, w.
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub half_extents: [f32; 3],
    pub radius: f32,
    pub density: f32,
    pub friction: f32,
    pub friction_combine_rule: u8,
    pub restitution: f32,
    pub restitution_combine_rule: u8,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub additional_solver_iterations: usize,
    pub can_sleep: bool,
    pub ccd: bool,
    pub soft_ccd_prediction: f32,
    pub sensor: bool,
}
impl Default for BodyDefinition {
    fn default() -> Self {
        Self {
            id: String::new(),
            fixed: false,
            shape: ShapeKind::Box,
            position: [0.0; 3],
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            half_extents: [0.5; 3],
            radius: 0.5,
            density: 1.0,
            friction: 0.5,
            friction_combine_rule: 0,
            restitution: 0.2,
            restitution_combine_rule: 0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            additional_solver_iterations: 0,
            can_sleep: true,
            ccd: false,
            soft_ccd_prediction: 0.0,
            sensor: false,
        }
    }
}
#[derive(Clone, Debug)]
struct BodyRecord {
    definition: BodyDefinition,
    body: RigidBodyHandle,
    collider: ColliderHandle,
}
#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    pub fixed: bool,
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub linear_velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
    pub sleeping: bool,
    pub enabled: bool,
}
/// Minimal deterministic world used by SceneSync's cross-platform profile.
pub struct World {
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    pipeline: PhysicsPipeline,
    islands: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    records: Vec<BodyRecord>,
    tick: u64,
}
impl World {
    pub fn new(gravity: [f32; 3], timestep: f32) -> Self {
        let integration_parameters = IntegrationParameters {
            dt: timestep,
            ..IntegrationParameters::default()
        };
        Self {
            gravity: vector![gravity[0], gravity[1], gravity[2]],
            integration_parameters,
            pipeline: PhysicsPipeline::new(),
            islands: IslandManager::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            records: Vec::new(),
            tick: 0,
        }
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn add_body(&mut self, definition: BodyDefinition) -> Result<(), &'static str> {
        if definition.id.is_empty() {
            return Err("stable id must not be empty");
        }
        if self
            .records
            .iter()
            .any(|record| record.definition.id == definition.id)
        {
            return Err("stable id must be unique");
        }
        let rotation = normalized_quaternion(definition.rotation);
        let pose = Isometry3::from_parts(
            Translation3::new(
                definition.position[0],
                definition.position[1],
                definition.position[2],
            ),
            rotation,
        );
        let mut body_builder = if definition.fixed {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .pose(pose)
        .linear_damping(definition.linear_damping.clamp(0.0, 1024.0))
        .angular_damping(definition.angular_damping.clamp(0.0, 1024.0))
        .gravity_scale(definition.gravity_scale.clamp(-1024.0, 1024.0))
        .can_sleep(definition.can_sleep)
        .additional_solver_iterations(definition.additional_solver_iterations)
        .soft_ccd_prediction(definition.soft_ccd_prediction.clamp(0.0, 1024.0));
        if !definition.fixed {
            body_builder = body_builder
                .linvel(vector![
                    definition.linear_velocity[0],
                    definition.linear_velocity[1],
                    definition.linear_velocity[2]
                ])
                .angvel(vector![
                    definition.angular_velocity[0],
                    definition.angular_velocity[1],
                    definition.angular_velocity[2]
                ])
                .ccd_enabled(definition.ccd);
        }
        let body = self.bodies.insert(body_builder);
        let collider_builder = match definition.shape {
            ShapeKind::Sphere => ColliderBuilder::ball(definition.radius.max(f32::MIN_POSITIVE)),
            ShapeKind::Box => ColliderBuilder::cuboid(
                definition.half_extents[0].max(f32::MIN_POSITIVE),
                definition.half_extents[1].max(f32::MIN_POSITIVE),
                definition.half_extents[2].max(f32::MIN_POSITIVE),
            ),
        }
        .density(definition.density.max(0.0))
        .friction(definition.friction.clamp(0.0, 4.0))
        .friction_combine_rule(combine_rule(definition.friction_combine_rule))
        .restitution(definition.restitution.clamp(0.0, 1.0))
        .restitution_combine_rule(combine_rule(definition.restitution_combine_rule))
        .sensor(definition.sensor);
        let collider = self
            .colliders
            .insert_with_parent(collider_builder, body, &mut self.bodies);
        self.records.push(BodyRecord {
            definition,
            body,
            collider,
        });
        Ok(())
    }

    pub fn remove_body(&mut self, id: &str) -> bool {
        let Some(index) = self
            .records
            .iter()
            .position(|record| record.definition.id == id)
        else {
            return false;
        };
        let record = self.records.remove(index);
        self.bodies.remove(
            record.body,
            &mut self.islands,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
        true
    }

    pub fn body_state(&self, id: &str) -> Option<BodyState> {
        let record = self
            .records
            .iter()
            .find(|record| record.definition.id == id)?;
        let body = self.bodies.get(record.body)?;
        let position = body.translation();
        let rotation = body.rotation().quaternion();
        let linear_velocity = if record.definition.fixed {
            [0.0; 3]
        } else {
            let value = body.linvel();
            [value.x, value.y, value.z]
        };
        let angular_velocity = if record.definition.fixed {
            [0.0; 3]
        } else {
            let value = body.angvel();
            [value.x, value.y, value.z]
        };
        Some(BodyState {
            fixed: record.definition.fixed,
            position: [position.x, position.y, position.z],
            rotation: [rotation.i, rotation.j, rotation.k, rotation.w],
            linear_velocity,
            angular_velocity,
            sleeping: body.is_sleeping(),
            enabled: body.is_enabled(),
        })
    }

    pub fn step(&mut self) {
        self.pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            &(),
            &(),
        );
        self.tick += 1;
    }

    pub fn step_to(&mut self, target_tick: u64) -> Result<(), &'static str> {
        if target_tick < self.tick {
            return Err("cannot step backwards");
        }
        while self.tick < target_tick {
            self.step();
        }
        Ok(())
    }

    pub fn canonical_state_hash(&self) -> String {
        format!("{:016x}", self.canonical_state_hash_u64())
    }

    fn canonical_state_hash_u64(&self) -> u64 {
        let mut records: Vec<_> = self.records.iter().collect();
        records.sort_by(|left, right| {
            stable_id_hash(&left.definition.id)
                .cmp(&stable_id_hash(&right.definition.id))
                .then_with(|| left.definition.id.cmp(&right.definition.id))
        });
        let mut hash = FNV64_OFFSET;
        hash = hash_string(hash, HASH_VERSION);
        hash = hash_string(hash, "rapier");
        hash = hash_string(hash, RAPIER_CORE_VERSION);
        hash = hash_vec3(hash, self.gravity.as_slice());
        hash = hash_f32(hash, self.integration_parameters.dt);
        hash = hash_u64(hash, INITIAL_NEXT_PID_CONTROLLER_ID);
        hash = hash_u64(hash, 0);
        hash = hash_u64(hash, records.len() as u64);
        for record in &records {
            hash = self.hash_body(hash, record);
        }
        hash = hash_u64(hash, records.len() as u64);
        for record in &records {
            hash = self.hash_collider(hash, record);
        }
        hash
    }

    fn hash_body(&self, mut hash: u64, record: &BodyRecord) -> u64 {
        let definition = &record.definition;
        let body = &self.bodies[record.body];
        let translation = body.translation();
        let rotation = body.rotation().quaternion();
        let linvel = if definition.fixed {
            [0.0; 3]
        } else {
            let value = body.linvel();
            [value.x, value.y, value.z]
        };
        let angvel = if definition.fixed {
            [0.0; 3]
        } else {
            let value = body.angvel();
            [value.x, value.y, value.z]
        };
        hash = hash_stable_identity(hash, &definition.id);
        hash = hash_u8(hash, definition.fixed as u8);
        hash = hash_f32(hash, body.gravity_scale());
        hash = hash_f32(hash, body.linear_damping());
        hash = hash_f32(hash, body.angular_damping());
        hash = hash_u64(hash, body.additional_solver_iterations() as u64);
        hash = hash_u8(hash, body.is_ccd_enabled() as u8);
        hash = hash_f32(hash, body.soft_ccd_prediction());
        hash = hash_u8(hash, definition.can_sleep as u8);
        hash = hash_vec3(hash, &[translation.x, translation.y, translation.z]);
        hash = hash_quat(hash, &[rotation.i, rotation.j, rotation.k, rotation.w]);
        hash = hash_vec3(hash, &linvel);
        hash = hash_vec3(hash, &angvel);
        hash = hash_u8(hash, body.is_sleeping() as u8);
        hash_u8(hash, body.is_enabled() as u8)
    }

    fn hash_collider(&self, mut hash: u64, record: &BodyRecord) -> u64 {
        let definition = &record.definition;
        let collider = &self.colliders[record.collider];
        hash = hash_stable_identity(hash, &definition.id);
        hash = hash_stable_identity(hash, &definition.id);
        hash = hash_u8(hash, 1);
        hash = hash_vec3(hash, &[0.0, 0.0, 0.0]);
        hash = hash_quat(hash, &[0.0, 0.0, 0.0, 1.0]);
        match definition.shape {
            ShapeKind::Sphere => {
                hash = hash_u8(hash, 1);
                hash = hash_f32(hash, definition.radius.max(f32::MIN_POSITIVE));
            }
            ShapeKind::Box => {
                hash = hash_u8(hash, 2);
                hash = hash_vec3(hash, &definition.half_extents);
            }
        }
        hash = hash_f32(hash, collider.density());
        hash = hash_f32(hash, collider.friction());
        hash = hash_u8(hash, collider.friction_combine_rule() as u8);
        hash = hash_f32(hash, collider.restitution());
        hash = hash_u8(hash, collider.restitution_combine_rule() as u8);
        hash = hash_u8(hash, collider.is_sensor() as u8);
        hash_u8(hash, collider.is_enabled() as u8)
    }
}
fn normalized_quaternion(value: [f32; 4]) -> UnitQuaternion<f32> {
    let quaternion = NaQuaternion::new(value[3], value[0], value[1], value[2]);
    if quaternion.norm_squared() > 0.0 && quaternion.norm_squared().is_finite() {
        UnitQuaternion::new_normalize(quaternion)
    } else {
        UnitQuaternion::identity()
    }
}
fn combine_rule(value: u8) -> CoefficientCombineRule {
    match value {
        1 => CoefficientCombineRule::Min,
        2 => CoefficientCombineRule::Multiply,
        3 => CoefficientCombineRule::Max,
        _ => CoefficientCombineRule::Average,
    }
}
pub fn stable_id_hash(value: &str) -> u64 {
    hash_bytes(FNV64_OFFSET, value.as_bytes())
}
fn hash_stable_identity(hash: u64, value: &str) -> u64 {
    hash_u64(hash_u8(hash, 1), stable_id_hash(value))
}
fn hash_bytes(mut hash: u64, bytes: &[u8]) -> u64 {
    for byte in bytes {
        hash = (hash ^ u64::from(*byte)).wrapping_mul(FNV64_PRIME);
    }
    hash
}
fn hash_u8(hash: u64, value: u8) -> u64 {
    hash_bytes(hash, &[value])
}
fn hash_u32(hash: u64, value: u32) -> u64 {
    hash_bytes(hash, &value.to_le_bytes())
}
fn hash_u64(hash: u64, value: u64) -> u64 {
    hash_bytes(hash, &value.to_le_bytes())
}
fn canonical_f32_bits(value: f32) -> u32 {
    if value == 0.0 {
        0
    } else if value.is_nan() {
        0x7fc00000
    } else {
        value.to_bits()
    }
}
fn hash_f32(hash: u64, value: f32) -> u64 {
    hash_u32(hash, canonical_f32_bits(value))
}
fn hash_string(hash: u64, value: &str) -> u64 {
    hash_bytes(hash_u32(hash, value.len() as u32), value.as_bytes())
}
fn hash_vec3(mut hash: u64, value: &[f32]) -> u64 {
    hash = hash_f32(hash, value[0]);
    hash = hash_f32(hash, value[1]);
    hash_f32(hash, value[2])
}
fn hash_quat(mut hash: u64, value: &[f32]) -> u64 {
    hash = hash_f32(hash, value[0]);
    hash = hash_f32(hash, value[1]);
    hash = hash_f32(hash, value[2]);
    hash_f32(hash, value[3])
}
/// Godot-facing owner for a SceneSync parity world.
#[derive(GodotClass)]
#[class(base = RefCounted)]
pub struct SceneSyncRapierWorld3D {
    world: World,
    last_error: GString,
    base: Base<RefCounted>,
}
#[godot_api]
impl IRefCounted for SceneSyncRapierWorld3D {
    fn init(base: Base<RefCounted>) -> Self {
        Self {
            world: World::new([0.0, -9.81, 0.0], 1.0 / 60.0),
            last_error: GString::new(),
            base,
        }
    }
}
#[godot_api]
impl SceneSyncRapierWorld3D {
    #[func]
    pub fn configure(&mut self, gravity: Vector3, timestep: f32) -> bool {
        if !gravity.x.is_finite() || !gravity.y.is_finite() || !gravity.z.is_finite() {
            self.last_error = "gravity must contain only finite values".into();
            return false;
        }
        if !timestep.is_finite() || timestep <= 0.0 {
            self.last_error = "timestep must be finite and positive".into();
            return false;
        }
        self.world = World::new([gravity.x, gravity.y, gravity.z], timestep);
        self.last_error = GString::new();
        true
    }

    /// Adds or replaces one body using the SceneSync wire-field names.
    #[func]
    pub fn add_body(&mut self, dictionary: VarDictionary) -> bool {
        let definition = match body_definition_from_dictionary(&dictionary) {
            Ok(definition) => definition,
            Err(error) => {
                self.last_error = error.into();
                return false;
            }
        };
        self.world.remove_body(&definition.id);
        match self.world.add_body(definition) {
            Ok(()) => {
                self.last_error = GString::new();
                true
            }
            Err(error) => {
                self.last_error = error.into();
                false
            }
        }
    }

    #[func]
    pub fn remove_body(&mut self, stable_id: GString) -> bool {
        self.world.remove_body(&stable_id.to_string())
    }

    #[func]
    pub fn get_body_state(&self, stable_id: GString) -> VarDictionary {
        let mut result = VarDictionary::new();
        let Some(state) = self.world.body_state(&stable_id.to_string()) else {
            return result;
        };
        #[cfg(feature = "scenesync-runtime")]
        result.set("id", &stable_id.to_variant());
        #[cfg(not(feature = "scenesync-runtime"))]
        result.set("id", stable_id);
        result.set("fixed", state.fixed);
        result.set(
            "position",
            Vector3::new(state.position[0], state.position[1], state.position[2]),
        );
        result.set(
            "rotation",
            Quaternion::new(
                state.rotation[0],
                state.rotation[1],
                state.rotation[2],
                state.rotation[3],
            ),
        );
        result.set(
            "linearVelocity",
            Vector3::new(
                state.linear_velocity[0],
                state.linear_velocity[1],
                state.linear_velocity[2],
            ),
        );
        result.set(
            "angularVelocity",
            Vector3::new(
                state.angular_velocity[0],
                state.angular_velocity[1],
                state.angular_velocity[2],
            ),
        );
        result.set("sleeping", state.sleeping);
        result.set("enabled", state.enabled);
        result
    }

    #[func]
    pub fn step_to(&mut self, target_tick: i64) -> bool {
        if target_tick < 0 {
            self.last_error = "target tick must not be negative".into();
            return false;
        }
        match self.world.step_to(target_tick as u64) {
            Ok(()) => {
                self.last_error = GString::new();
                true
            }
            Err(error) => {
                self.last_error = error.into();
                false
            }
        }
    }

    #[func]
    pub fn get_tick(&self) -> i64 {
        self.world.tick().min(i64::MAX as u64) as i64
    }

    #[func]
    pub fn get_canonical_state_hash(&self) -> GString {
        let hash = self.world.canonical_state_hash();
        GString::from(hash.as_str())
    }

    #[func]
    pub fn get_profile(&self) -> GString {
        PROFILE.into()
    }

    #[func]
    pub fn get_rapier_core_version(&self) -> GString {
        RAPIER_CORE_VERSION.into()
    }

    #[func]
    pub fn get_hash_version(&self) -> GString {
        HASH_VERSION.into()
    }

    #[func]
    pub fn get_last_error(&self) -> GString {
        self.last_error.clone()
    }
}
fn body_definition_from_dictionary(
    dictionary: &VarDictionary,
) -> Result<BodyDefinition, &'static str> {
    let id = dictionary_string(dictionary, "id").ok_or("stable id must be a non-empty string")?;
    if id.is_empty() {
        return Err("stable id must be a non-empty string");
    }
    let density = dictionary_f32(dictionary, "density", 1.0);
    let body_type = dictionary_string(dictionary, "type").unwrap_or_default();
    let fixed =
        body_type == "fixed" || dictionary_bool(dictionary, "static", false) || density == 0.0;
    let shape = if dictionary_string(dictionary, "shape").as_deref() == Some("sphere") {
        ShapeKind::Sphere
    } else {
        ShapeKind::Box
    };
    let linear_velocity = dictionary_vec3(dictionary, "linearVelocity")
        .or_else(|| dictionary_vec3(dictionary, "velocity"))
        .unwrap_or([0.0; 3]);
    Ok(BodyDefinition {
        id,
        fixed,
        shape,
        position: dictionary_vec3(dictionary, "position").unwrap_or([0.0; 3]),
        rotation: dictionary_quat(dictionary, "rotation").unwrap_or([0.0, 0.0, 0.0, 1.0]),
        linear_velocity,
        angular_velocity: dictionary_vec3(dictionary, "angularVelocity").unwrap_or([0.0; 3]),
        half_extents: dictionary_vec3(dictionary, "halfExtents").unwrap_or([0.5; 3]),
        radius: dictionary_f32(dictionary, "radius", 0.5),
        density: density.max(0.0),
        friction: dictionary_f32(dictionary, "friction", 0.5),
        friction_combine_rule: dictionary_i64(dictionary, "frictionCombineRule", 0).clamp(0, 3)
            as u8,
        restitution: dictionary_f32(dictionary, "restitution", 0.2),
        restitution_combine_rule: dictionary_i64(dictionary, "restitutionCombineRule", 0)
            .clamp(0, 3) as u8,
        linear_damping: dictionary_f32(dictionary, "linearDamping", 0.0),
        angular_damping: dictionary_f32(dictionary, "angularDamping", 0.0),
        gravity_scale: dictionary_f32(dictionary, "gravityScale", 1.0),
        additional_solver_iterations: dictionary_i64(dictionary, "additionalSolverIterations", 0)
            .max(0) as usize,
        can_sleep: dictionary_bool(dictionary, "canSleep", true),
        ccd: dictionary_bool(dictionary, "ccd", false),
        soft_ccd_prediction: dictionary_f32(dictionary, "softCcdPrediction", 0.0),
        sensor: dictionary_bool(dictionary, "sensor", false),
    })
}
fn dictionary_string(dictionary: &VarDictionary, key: &str) -> Option<String> {
    dictionary
        .get(key)
        .and_then(|value| value.try_to::<GString>().ok())
        .map(|value| value.to_string())
}
fn dictionary_bool(dictionary: &VarDictionary, key: &str, fallback: bool) -> bool {
    dictionary
        .get(key)
        .and_then(|value| value.try_to::<bool>().ok())
        .unwrap_or(fallback)
}
fn variant_f32(value: &Variant) -> Option<f32> {
    value
        .try_to::<f64>()
        .ok()
        .map(|number| number as f32)
        .or_else(|| value.try_to::<i64>().ok().map(|number| number as f32))
        .filter(|number| number.is_finite())
}
fn dictionary_f32(dictionary: &VarDictionary, key: &str, fallback: f32) -> f32 {
    dictionary
        .get(key)
        .as_ref()
        .and_then(variant_f32)
        .unwrap_or(fallback)
}
fn dictionary_i64(dictionary: &VarDictionary, key: &str, fallback: i64) -> i64 {
    dictionary
        .get(key)
        .and_then(|value| value.try_to::<i64>().ok())
        .unwrap_or(fallback)
}
fn dictionary_vec3(dictionary: &VarDictionary, key: &str) -> Option<[f32; 3]> {
    let value = dictionary.get(key)?;
    if let Ok(vector) = value.try_to::<Vector3>() {
        return Some([vector.x, vector.y, vector.z]);
    }
    let array = value.try_to::<Array<Variant>>().ok()?;
    if array.len() < 3 {
        return None;
    }
    Some([
        variant_f32(&array.at(0))?,
        variant_f32(&array.at(1))?,
        variant_f32(&array.at(2))?,
    ])
}
fn dictionary_quat(dictionary: &VarDictionary, key: &str) -> Option<[f32; 4]> {
    let value = dictionary.get(key)?;
    if let Ok(quaternion) = value.try_to::<Quaternion>() {
        return Some([quaternion.x, quaternion.y, quaternion.z, quaternion.w]);
    }
    let array = value.try_to::<Array<Variant>>().ok()?;
    if array.len() < 4 {
        return None;
    }
    Some([
        variant_f32(&array.at(0))?,
        variant_f32(&array.at(1))?,
        variant_f32(&array.at(2))?,
        variant_f32(&array.at(3))?,
    ])
}
#[cfg(test)]
mod tests {
    use super::*;
    fn box_body(id: &str, fixed: bool, position: [f32; 3], density: f32) -> BodyDefinition {
        BodyDefinition {
            id: id.to_owned(),
            fixed,
            position,
            density,
            ..BodyDefinition::default()
        }
    }
    fn run_fixture(mut world: World, expected: &[(u64, &str)]) {
        for (tick, expected_hash) in expected {
            world.step_to(*tick).unwrap();
            assert_eq!(
                world.canonical_state_hash(),
                *expected_hash,
                "canonical hash mismatch at tick {tick}"
            );
        }
    }
    #[test]
    fn freefall_fixture_matches_browser() {
        let mut world = World::new([0.0, -9.81, 0.0], 1.0 / 60.0);
        world
            .add_body(BodyDefinition {
                id: "box-1".to_owned(),
                position: [-0.75, 5.0, 0.0],
                linear_velocity: [0.75, 0.0, 0.15],
                angular_velocity: [0.35, 1.25, 0.55],
                linear_damping: 0.02,
                angular_damping: 0.02,
                can_sleep: false,
                ..BodyDefinition::default()
            })
            .unwrap();
        run_fixture(
            world,
            &[
                (0, "1a8cf55faa0e4e4e"),
                (1, "8f9f11fbf1f52663"),
                (2, "a882f0aedd1ea2e3"),
                (10, "b05d71580dd8b483"),
                (60, "14f6c93758a3967a"),
                (120, "165dfa5582a4ba24"),
                (300, "62275fdf452d3e1b"),
                (600, "7e02eccebb676aad"),
            ],
        );
    }
    #[test]
    fn basic_contact_fixture_matches_browser() {
        let mut world = World::new([0.0, -9.81, 0.0], 1.0 / 60.0);
        world
            .add_body(BodyDefinition {
                half_extents: [6.0, 0.5, 6.0],
                friction: 0.0,
                restitution: 0.0,
                ..box_body("floor", true, [0.0, -0.5, 0.0], 0.0)
            })
            .unwrap();
        world
            .add_body(BodyDefinition {
                friction: 0.0,
                restitution: 0.0,
                can_sleep: false,
                ..box_body("box-1", false, [0.0, 5.0, 0.0], 1.0)
            })
            .unwrap();
        run_fixture(
            world,
            &[
                (0, "717960f5748ebc9b"),
                (1, "97be88421da8e037"),
                (2, "9a569c50cfad59f8"),
                (10, "7739acdd722fa024"),
                (30, "a4e8bcc70f15dfc3"),
                (55, "38956d3b881e76e2"),
                (56, "dd9c85319711aa62"),
                (57, "1e60f60ea96e5965"),
                (58, "f53842bcbbdae1e8"),
                (60, "66ee8d6b45f00a51"),
                (120, "c49b9e20d1703bfe"),
                (300, "8e235a1c14a6d011"),
                (600, "1d1d479bf100e287"),
            ],
        );
    }
    #[test]
    fn rotating_contact_fixture_matches_browser() {
        let mut world = World::new([0.0, -9.81, 0.0], 1.0 / 60.0);
        world
            .add_body(BodyDefinition {
                half_extents: [6.0, 0.5, 6.0],
                ..box_body("floor", true, [0.0, -0.5, 0.0], 0.0)
            })
            .unwrap();
        world
            .add_body(BodyDefinition {
                position: [-0.75, 5.0, 0.0],
                linear_velocity: [0.75, 0.0, 0.15],
                angular_velocity: [0.35, 1.25, 0.55],
                linear_damping: 0.02,
                angular_damping: 0.02,
                can_sleep: false,
                ..box_body("box-1", false, [-0.75, 5.0, 0.0], 1.0)
            })
            .unwrap();
        run_fixture(
            world,
            &[
                (0, "43af70bb0d584167"),
                (1, "65fef4a4d29b40ba"),
                (2, "52649f6bbd3540c2"),
                (10, "f2bf5f533b788f16"),
                (60, "0a16d338571a280c"),
                (120, "7531c543fd7cf7fa"),
                (300, "ba91b9785cf9168c"),
                (600, "ba91b9785cf9168c"),
            ],
        );
    }
}
