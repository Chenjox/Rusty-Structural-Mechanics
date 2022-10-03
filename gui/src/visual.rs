use std::f64::consts;
use structmech::stiffness::system::*;

pub trait Visualizeable {
    fn visualize(&self) -> String;
}

impl Visualizeable for System {
    fn visualize(&self) -> String {
        let mut s = String::new();

        s.push_str("\\begin{tikzpicture}\n");
        for bi in 0..self.get_points().len() {
            let p = self.get_points()[bi];
            s.push_str(&format!("  \\point{{p{}}}{{{}}}{{{}}}\n", bi, p.x, p.y));
        }
        // Balken
        for bi in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(bi);
            let to = self.get_beam_to_point(bi);

            let alph = self.get_beam_alpha(bi);

            //println!("{}", alph / consts::PI * 180.0);

            s.push_str(&format!("  \\beam{{1}}{{p{}}}{{p{}}}\n", from, to));
        }
        // Supports
        for supi in 0..self.get_supports().len() {
            let p = self.get_support_points()[supi];
            let sup = &self.get_supports()[supi];
            let alpha = sup.get_alpha() / consts::PI * 180.0;

            let mut t = 3;

            let dofs = sup.get_free_dofs();
            // Auswahl der Supportart.
            if dofs[2] {
                // Momentennullfeld
                if dofs[0] {
                    t = 2;
                } else {
                    t = 1;
                }
            } else {
                // kein Momentennullfeld
                if dofs[0] {
                    t = 4;
                } else {
                    t = 7;
                }
            }

            if t == 7 {
                s.push_str(&format!("  \\support{{4}}{{p{}}}[{1:.4}+90]\n", p, alpha));
                s.push_str(&format!("  \\support{{4}}{{p{}}}[{1:.4}-90]\n", p, alpha));
            } else {
                s.push_str(&format!("  \\support{{{}}}{{p{}}}[{2:.4}]\n", t, p, alpha));
            }
        }
        s.push_str("\\end{tikzpicture}\n");

        return s;
    }
}
