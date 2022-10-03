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
        for bi in 0..self.get_beams().len() {
            let from = self.get_beam_from_point(bi);
            let to = self.get_beam_to_point(bi);

            let alph = self.get_beam_alpha(bi);

            println!("{}", alph / consts::PI * 180.0);

            s.push_str(&format!("  \\beam{{1}}{{p{}}}{{p{}}}\n", from, to));
        }
        s.push_str("\\end{tikzpicture}\n");

        return s;
    }
}
