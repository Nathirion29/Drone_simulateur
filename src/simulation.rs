use nalgebra::Vector2;

/// État du drone en 2D
pub struct State {
    pub pos: Vector2<f64>,   // [x, z]
    pub vel: Vector2<f64>,   // [vx, vz]
    pub mass: f64,
    pub gravity: f64,
}

impl State {
    pub fn new(mass: f64, gravity: f64) -> Self {
        Self {
            pos: Vector2::new(0.0, 0.0),
            vel: Vector2::new(0.0, 0.0),
            mass,
            gravity,
        }
    }
}

/// Effectue un pas d’intégration (Euler explicite)
/// thrust : poussée verticale positive
/// dt     : pas de temps (s)
pub fn step(state: &mut State, thrust: f64, dt: f64) {
    // Force nette verticale = thrust - poids
    let fz = thrust - state.mass * state.gravity;
    // accélération
    let az = fz / state.mass;
    // mise à jour de la vitesse et de la position
    state.vel.y += az * dt;
    state.pos.y += state.vel.y * dt;
    // pour l’horizontale, on peut simuler un drift nul (ou simple modèle si besoin)
    state.vel.x = 0.0;
    state.pos.x += state.vel.x * dt;
}
