use std::f64::consts;
use structmech::stiffness::system::*;

pub fn visualize_asymptote(sys: &System) -> String {
    let mut s = String::new();
    s.push_str(
        "settings.outformat = \"pdf\";
unitsize(1cm);

// Um die Auflager zu zeichnen.
void auflager(pair z, bool[] dof, real degrees) {
  if(!dof[0]){
    draw(rotate(degrees,z)*(z+(0.0,0.25) -- z+(-0.0,-0.25)), black + linewidth(0.5mm));
  }
  if(!dof[1]){
    draw(rotate(degrees,z)*(z+(0.25,0.0) -- z+(-0.25,-0.0)), black + linewidth(0.5mm));
  }
  if(dof[2]){
    filldraw(circle(z,0.1),white);
    draw(circle(z,0.1),black);
  }else{
    filldraw(box(z+(-0.1,-0.1), z+(0.1,0.1)),black);
  }
}

//Um Balken darzustellen
void balken(pair z1, pair z2) {
  //die normale Linie
  path p = (z1 -- z2);
  draw(p,black + linewidth(0.5mm));
  //die gestrichelte
  //draw(shift()*p,black + linewidth(0.5mm));
  }
",
    );
    for bi in 0..sys.get_points().len() {
        let p = sys.get_points()[bi];
        s.push_str(&format!("pair p{} = ({},{});\n", bi, p.x, p.y));
    }

    for bi in 0..sys.get_beams().len() {
        let from = sys.get_beam_from_point(bi);
        let to = sys.get_beam_to_point(bi);

        let alph = sys.get_beam_alpha(bi);

        //println!("{}", alph / consts::PI * 180.0);

        s.push_str(&format!("balken(p{},p{});\n", from, to));
    }
    for supi in 0..sys.get_supports().len() {
        let p = sys.get_support_points()[supi];
        let sup = &sys.get_supports()[supi];
        let alpha = sup.get_alpha() / consts::PI * 180.0;

        let dofs = sup.get_free_dofs();
        // Auswahl der Supportart.
        s.push_str(&format!(
            "bool[] auf{} = {{ {},{},{} }};\n",
            supi, dofs[0], dofs[1], dofs[2]
        ));
        s.push_str(&format!("auflager(p{},auf{},{});\n", p, supi, alpha));
    }
    return s;
}

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
