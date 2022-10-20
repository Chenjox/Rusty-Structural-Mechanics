use std::f64::consts;
use structmech::stiffness::direct_stiffness::BeamResultSet;
use structmech::stiffness::system::*;

pub fn visualize_asymptote(sys: &System) -> String {
    let mut s = String::new();
    s.push_str(
        "settings.outformat = \"pdf\";
unitsize(1cm);

// Um die Auflager zu zeichnen.
void auflager(pair z, bool[] dof, real degrees) {
  if(dof[0]){
    draw(rotate(degrees,z)*(z+(0.0,0.25) -- z+(-0.0,-0.25)), black + linewidth(0.5mm));
  }
  if(dof[1]){
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
void balken(pair z1, pair z2, real degrees) {
  //die normale Linie
  path p = (z1 -- z2);
  draw(p,black + linewidth(0.5mm));
  //die gestrichelte
  path p = (rotate(degrees,z1)*shift(0.05,-0.05)*z1 -- rotate(degrees,z2)*shift(-0.05,-0.05)*z2);
  draw(p,black + linewidth(0.3mm)+dashed);
}

// Für die Stabanschlüsse
void anschluss(pair z1, pair z2, real beam_degree, bool[] dof, real dof_degree){
  pair dir = unit(z2 - z1);
  if(dof[0]){
    pair r1 = z2 - 0.1*dir;
    fill(rotate(beam_degree+dof_degree+90,r1)*box(r1 + (0.05,0.1),r1 - (0.05,0.1)),white);
    draw(rotate(beam_degree+dof_degree+90,r1)*( r1 + (0.05,0.1) -- r1 + (0.05,-0.1) ));
    draw(rotate(beam_degree+dof_degree+90,r1)*( r1 + (-0.05,0.1) -- r1 + (-0.05,-0.1) ));
  }
  if(dof[1]){
    pair r1 = z2 - 0.1*dir;
    fill(rotate(beam_degree+dof_degree,r1)*box(r1 + (0.05,0.1),r1 - (0.05,0.1)),white);
    draw(rotate(beam_degree+dof_degree,r1)*( r1 + (0.05,0.1) -- r1 + (0.05,-0.1) ));
    draw(rotate(beam_degree+dof_degree,r1)*( r1 + (-0.05,0.1) -- r1 + (-0.05,-0.1) ));
  }
  if(dof[2]){
    pair z = z2 - 0.1*dir;
    filldraw(circle(z,0.1),white);
    draw(circle(z,0.1),black);
  }
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

        let alph = sys.get_beam_alpha(bi) / consts::PI * 180.0;

        //println!("{}", alph / consts::PI * 180.0);

        s.push_str(&format!("balken(p{},p{},{});\n", from, to, alph));
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
    for bi in 0..sys.get_beams().len() {
        let from = sys.get_beam_from_point(bi);
        let to = sys.get_beam_to_point(bi);
        let dofs = sys.get_beams()[bi].get_dofs();
        let from_alpha = sys.get_beams()[bi].get_start_alpha();
        let to_alpha = sys.get_beams()[bi].get_end_alpha();

        let alph = sys.get_beam_alpha(bi) / consts::PI * 180.0;

        s.push_str(&format!(
            "bool[] ans1{} = {{ {},{},{} }};\n",
            bi, dofs[0], dofs[1], dofs[2]
        ));
        s.push_str(&format!(
            "bool[] ans2{} = {{ {},{},{} }};\n",
            bi, dofs[3], dofs[4], dofs[5]
        ));
        s.push_str(&format!(
            "anschluss(p{},p{},{}, ans1{}, {});\n",
            to, from, alph, bi, to_alpha
        ));
        s.push_str(&format!(
            "anschluss(p{},p{},{}, ans2{}, {});\n",
            from, to, alph, bi, from_alpha
        ));

        //println!("{}", alph / consts::PI * 180.0);
    }
    return s;
}

pub fn visualize_result_asymptote(
    sys: &System,
    res: &BeamResultSet,
    samples: u64,
    dof_s: usize,
) -> String {
    let mut s = String::new();
    s.push_str(&visualize_asymptote(sys));

    // Zuerst müssen wir alle Werte in Erfahrung bringen.
    let mut resultsMoment = Vec::new();
    for b in 0..res.get_results().len() {
        let resu = &res.get_results()[b];
        let l = resu.get_beam_lenght();

        let mut moment = Vec::new();
        for sam in 0..=samples {
            let inter = sam as f64 / samples as f64;
            let m = resu.get_internals_at(l * inter)[dof_s];
            moment.push(m);
        }
        resultsMoment.push(moment);
    }
    // Anschließend müssen wir das Maximum in erfahrung brigen.
    let mut max = 0.0_f64;
    for m in resultsMoment {
        for f in m {
            max = max.max(f.abs());
        }
    }
    // Dessen Kehrwert der Skalierer ist.
    let r = 1.0 / max;
    // Jetzt der zweite durchlauf
    for b in 0..res.get_results().len() {
        let beam = sys.get_beams()[b];
        let resu = &res.get_results()[b];
        let l = resu.get_beam_lenght();

        let start = sys.get_beam_from_point(b);
        let alph = sys.get_beam_alpha(b) / consts::PI * 180.0;

        s.push_str(&format!("path b{} = (0.0,0.0) --", b));

        for sam in 0..=samples {
            let inter = sam as f64 / samples as f64;
            let m = resu.get_internals_at(l * inter)[dof_s];
            s.push_str(&format!("({0:.5},{1:.5}) -- ", l * inter, m * r));
        }
        s.push_str(&format!("({},0.0) -- cycle;\n", l));

        s.push_str(&format!(
            "draw(rotate({},p{})*shift(p{})*b{},red);\n",
            alph, start, start, b
        ));
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
