use bevy::{
    prelude::*,
    DefaultPlugins,
    sprite::{Wireframe2dPlugin},
    diagnostic::FrameTimeDiagnosticsPlugin,
};

use crate::boids::BoidsPlugin;
use crate::frame_counter::FpsPlugin;

mod boids;
mod frame_counter;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, Wireframe2dPlugin, FrameTimeDiagnosticsPlugin))
        .add_plugins((BoidsPlugin::default(), FpsPlugin))
        .run();
}
