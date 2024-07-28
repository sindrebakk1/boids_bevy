use std::{
    ops::{AddAssign, Div, Mul, MulAssign, Sub},
    f32::consts::{PI, TAU}
};
use bevy::{
    app::{App, Plugin},
    prelude::{
        Component,
        Vec2,
        Commands,
        Bundle,
        Query,
        With,
        Resource,
        Entity,
        Res,
        ResMut,
        Startup,
        Update,
        IntoSystemConfigs,
        Transform,
        Vec3,
        Color,
        Mesh,
        Assets,
        Handle,
        Triangle2d,
        Camera2dBundle,
        Window,
        Quat,
        Time
    },
    sprite::{ColorMaterial, MaterialMesh2dBundle, Mesh2dHandle},
};
use rand::prelude::{StdRng};
use rand::{Rng, SeedableRng};

const DEFAULT_MAX_BOID_COUNT: u32 = 600;

const R: f32 = 5.;

const MAX_FORCE: f32 = 5.0;
const MAX_SPEED: f32 = 300.0;

const DESIRED_SEPARATION: f32 = 50.;
const NEIGHBOUR_RADIUS: f32 = 100.;

const SEPARATION_MULTIPLIER: f32 = 1.2;
const ALIGN_MULTIPLIER: f32 = 1.0;
const COHESION_MULTIPLIER: f32 = 1.0;

#[derive(Component)]
struct Position(Vec2);

#[derive(Component)]
struct Velocity(Vec2);

#[derive(Component)]
struct Acceleration(Vec2);

#[derive(Component)]
struct Boid {
    max_force: f32,
    max_speed: f32,
}

impl Default for Boid {
    fn default() -> Self {
        Boid {
            max_force: MAX_FORCE,
            max_speed: MAX_SPEED,
        }
    }
}

impl Boid {
    fn seek(&self, target: Vec2, position: &Position, velocity: &Velocity) -> Vec2 {
        target
            .sub(position.0)
            .normalize()
            .mul(self.max_speed)
            .sub(velocity.0)
            .clamp_length_max(self.max_force)
    }

    fn separate(
        &self,
        position: &Position,
        velocity: &Velocity,
        boids: &Res<Boids>,
        positions: &Query<&Position>
    ) -> Vec3 {
        let mut steer = Vec3::ZERO;
        let mut count = 0;
        for &boid in &boids.0 {
            if let Ok(pos) = positions.get(boid) {
                let dist = position.0.distance(pos.0);
                if dist > 0f32 && dist < DESIRED_SEPARATION {
                    let diff = position.0
                        .sub(pos.0)
                        .normalize()
                        .div(dist);
                    steer.add_assign(Vec3::from((diff, 0.)));
                    count += 1;
                }
            }
        }
        if count > 0 {
            steer = steer.div(count as f32);
        }
        if steer.length() > 0f32 {
            steer = steer
                .normalize()
                .mul(self.max_speed)
                .sub(Vec3::from((velocity.0, 0.)));
            steer = steer.clamp_length_max(self.max_force);
        }
        steer
    }

    fn align(
        &self,
        position: &Position,
        velocity: &Velocity,
        boids: &Res<Boids>,
        positions: &Query<&Position>,
        velocities: &Query<&Velocity>
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;
        for &boid in &boids.0 {
            if let (
                Ok(pos),
                Ok(vel)
            ) = (
                positions.get(boid),
                velocities.get(boid)
            ) {
                let dist = position.0.distance(pos.0);
                if dist > 0f32 && dist < NEIGHBOUR_RADIUS {
                    sum.add_assign(vel.0);
                    count += 1;
                }
            }
        }
        if count > 0 {
            sum.div(count as f32)
                .normalize()
                .mul(self.max_speed)
                .sub(velocity.0)
                .clamp_length_max(self.max_force)
        } else {
            Vec2::new(0., 0.)
        }
    }

    fn cohesion(
        &self,
        position: &Position,
        velocity: &Velocity,
        boids: &Res<Boids>,
        positions: &Query<&Position>
    ) -> Vec2 {
        let mut sum = Vec2::ZERO;
        let mut count = 0;
        for &boid in &boids.0 {
            if let Ok(pos) = positions.get(boid) {
                let dist = position.0.distance(pos.0);
                if dist > 0f32 && dist < NEIGHBOUR_RADIUS {
                    sum.add_assign(pos.0);
                    count += 1;
                }
            }
        }
        if count > 0 {
            self.seek(sum.div(count as f32), position, velocity)
        } else {
            Vec2::new(0., 0.)
        }
    }
}

#[derive(Bundle)]
pub struct BoidBundle<T: Bundle> {
    marker: Boid,
    position: Position,
    velocity: Velocity,
    acceleration: Acceleration,
    mesh: T,
}

// list of spawned boids that is updated runtime
#[derive(Resource)]
struct Boids(Vec<Entity>);

impl Default for Boids {
    fn default() -> Self {
        Boids(Vec::new())
    }
}

#[derive(Resource)]
struct RandomGenerator {
    rng: StdRng,
}

impl RandomGenerator {
    fn new(seed: [u8; 32]) -> Self {
        RandomGenerator {
            rng: StdRng::from_seed(seed),
        }
    }

    fn random_f32(&mut self, range: std::ops::Range<f32>) -> f32 {
        self.rng.gen_range(range)
    }
}

#[derive(Resource)]
struct BoidMesh(Mesh2dHandle);

#[derive(Resource)]
struct BoidMaterial(Handle<ColorMaterial>);

#[derive(Resource)]
struct MaxBoidCount(u32);

#[derive(Resource)]
struct BoidCount(u32);

impl Default for BoidCount {
    fn default() -> Self {
        BoidCount(0)
    }
}

pub struct BoidsPlugin {
    max_boid_count: u32
}

impl BoidsPlugin {
    pub(crate) fn new(max_boid_count: u32) -> Self {
        BoidsPlugin {
            max_boid_count
        }
    }

    pub(crate) fn default() -> Self {
        BoidsPlugin {
            max_boid_count: DEFAULT_MAX_BOID_COUNT,
        }
    }
}

impl Plugin for BoidsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Boids>()
            .init_resource::<BoidCount>()
            .insert_resource(MaxBoidCount(self.max_boid_count))
            .add_systems(Startup, (setup).chain())
            .add_systems(Update, spawn)
            .add_systems(Update, (flock, update_boid).chain());
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn(Camera2dBundle::default());

    let seed = [0u8;32];
    commands.insert_resource(RandomGenerator::new(seed));
    
    commands.insert_resource(BoidMesh(Mesh2dHandle(meshes.add(Triangle2d::new(
        Vec2::Y * 6.,
        Vec2::new(-3., -3.),
        Vec2::new(3., -3.)
    )))));

    commands.insert_resource(BoidMaterial(materials.add(Color::WHITE)));
}

fn spawn(
    mut commands: Commands,
    mut boids: ResMut<Boids>,
    mesh: Res<BoidMesh>,
    material: Res<BoidMaterial>,
    mut rng: ResMut<RandomGenerator>,
    max_boid_count: Res<MaxBoidCount>,
    mut boid_count: ResMut<BoidCount>,
) {
    if boid_count.0 < max_boid_count.0 {
        let a = rng.random_f32(0.0..TAU);
        let boid = BoidBundle {
            marker: Default::default(),
            position: Position(Vec2::ZERO),
            velocity: Velocity(Vec2::new(a.cos(), a.sin()).mul(MAX_SPEED/2.0)),
            acceleration: Acceleration(Vec2::ZERO),
            mesh: MaterialMesh2dBundle {
                mesh: mesh.0.clone(),
                material: material.0.clone(),
                ..Default::default()
            },
        };
        let boid_id = commands.spawn(boid).id();
        boids.0.push(boid_id);
        boid_count.0 += 1;
    }
}

fn flock(
    mut query: Query<(&Position, &Velocity, &mut Acceleration, &Boid), With<Boid>>,
    positions: Query<&Position>,
    velocities: Query<&Velocity>,
    boids: Res<Boids>,
) {
    for (pos, vel, mut acc, boid) in query.iter_mut() {
        let sep = boid.separate(pos, vel, &boids, &positions)
            .mul(SEPARATION_MULTIPLIER); // Separation
        let ali = boid.align(pos, vel, &boids, &positions, &velocities)
            .mul(ALIGN_MULTIPLIER); // Alignment
        let coh = boid.cohesion(pos, vel, &boids, &positions)
            .mul(COHESION_MULTIPLIER); // Cohesion

        acc.0.add_assign(Vec2::from((sep.x, sep.y)));
        acc.0.add_assign(ali);
        acc.0.add_assign(coh);
    }
}

fn update_boid(
    mut query: Query<(
        &mut Position,
        &mut Velocity,
        &mut Acceleration,
        &mut Transform, &Boid
    ), With<Boid>>,
    mut windows: Query<&mut Window>,
    time: Res<Time>
) {
    let window = windows.single_mut();
    let half_width = window.width() / 2.0;
    let half_height = window.height() / 2.0;
    for (
        mut pos,
        mut vel,
        mut acc,
        mut transform,
        boid
    ) in query.iter_mut() {
        let theta = vel.0.y.atan2(vel.0.x) + (90. * PI / 180.) * -1.;
        transform.translation = Vec3::new(pos.0.x, pos.0.y, 0.);
        transform.rotation = Quat::from_rotation_z(theta);

        // update velocity
        vel.0.add_assign(acc.0);
        // limit speed
        vel.0 = vel.0.clamp_length_max(boid.max_speed);
        // update position
        pos.0.add_assign(vel.0 * time.delta_seconds());

        // Wrap around the x-axis
        if pos.0.x < -half_width - R {
            pos.0.x = half_width + R;
        } else if pos.0.x > half_width + R {
            pos.0.x = -half_width - R;
        }

        // Wrap around the y-axis
        if pos.0.y < -half_height - R {
            pos.0.y = half_height + R;
        } else if pos.0.y > half_height + R {
            pos.0.y = -half_height - R;
        }

        // reset acceleration to 0
        acc.0.mul_assign(0f32);
    }
}
