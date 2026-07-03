use crate::primitives::Sdf;
use opensolid_core::types::Point3;

pub struct SmoothUnion<A, B> {
    pub a: A,
    pub b: B,
    pub radius: f64,
}

impl<A: Sdf, B: Sdf> Sdf for SmoothUnion<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        let da = self.a.eval(p);
        let db = self.b.eval(p);
        let h = (0.5 + 0.5 * (db - da) / self.radius).clamp(0.0, 1.0);
        db * (1.0 - h) + da * h - self.radius * h * (1.0 - h)
    }
}

pub struct SmoothSubtraction<A, B> {
    pub a: A,
    pub b: B,
    pub radius: f64,
}

impl<A: Sdf, B: Sdf> Sdf for SmoothSubtraction<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        let da = self.a.eval(p);
        let db = -self.b.eval(p);
        let h = (0.5 - 0.5 * (db + da) / self.radius).clamp(0.0, 1.0);
        da * (1.0 - h) + db * h + self.radius * h * (1.0 - h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::Sphere;

    #[test]
    fn smooth_union_blends() {
        let a = Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let b = Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let su = SmoothUnion { a, b, radius: 0.3 };
        // Smooth union should be more negative at origin than sharp union
        let sharp_a = Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let sharp_b = Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let sharp = sharp_a
            .eval(&Point3::origin())
            .min(sharp_b.eval(&Point3::origin()));
        assert!(su.eval(&Point3::origin()) < sharp);
    }
}
