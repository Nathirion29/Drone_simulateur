/// Contrôleur PID générique
pub struct PID {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub integral: f64,
    pub prev_error: f64,
    pub output_min: f64,
    pub output_max: f64,
}

impl PID {
    pub fn new(kp: f64, ki: f64, kd: f64, out_min: f64, out_max: f64) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_error: 0.0, output_min: out_min, output_max: out_max }
    }

    /// Calcule la commande PID
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        // terme intégral
        self.integral += error * dt;
        // terme dérivé
        let derivative = (error - self.prev_error) / dt;
        // somme pondérée
        let mut output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        // saturation
        if output > self.output_max { output = self.output_max; }
        if output < self.output_min { output = self.output_min; }
        self.prev_error = error;
        output
    }
}
