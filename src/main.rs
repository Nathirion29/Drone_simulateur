mod simulation;
mod controller;

use simulation::{State, step};
use controller::PID;
use std::fs::File;
use csv::Writer;
use chrono::Utc;

fn main() {
    // Paramètres
    let dt      = 0.01;      // pas de temps (s)
    let t_end   = 10.0;      // durée totale (s)
    let z_target= 5.0;       // altitude cible (m)
    let mass    = 1.0;       // kg
    let gravity = 9.81;      // m/s²

    let mut state = State::new(mass, gravity);
    let mut pid   = PID::new(2.0, 0.5, 1.0, 0.0, 20.0);

    // Fichier CSV pour l’export
    let mut wtr = Writer::from_writer(File::create("flight_data.csv").unwrap());
    wtr.write_record(&["time","x","z","vx","vz","thrust"]).unwrap();

    let mut time = 0.0;
    while time <= t_end {
        let error  = z_target - state.pos.y;
        let thrust = pid.update(error, dt);

        // Simulation
        step(&mut state, thrust, dt);

        // Écriture CSV
        wtr.write_record(&[
            &format!("{:.3}", time),
            &format!("{:.3}", state.pos.x),
            &format!("{:.3}", state.pos.y),
            &format!("{:.3}", state.vel.x),
            &format!("{:.3}", state.vel.y),
            &format!("{:.3}", thrust),
        ]).unwrap();

        time += dt;
    }
    wtr.flush().unwrap();

    println!("Simulation terminée à {} s, données dans flight_data.csv", t_end);
}
