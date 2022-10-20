settings.outformat = "pdf";
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
void balken(pair z1, pair z2, real degrees) {
  //die normale Linie
  path p = (z1 -- z2);
  draw(p,black + linewidth(0.5mm));
  //die gestrichelte
  path p = (rotate(degrees,z1)*shift(0.05,-0.05)*z1 -- rotate(degrees,z2)*shift(-0.05,-0.05)*z2);
  draw(p,black + linewidth(0.3mm)+dashed);
}

//Um die Anschlüsse darzustellen
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
pair p0 = (0,0);
pair p1 = (3,4);
pair p2 = (9,4);
balken(p0,p1,53.13010235415599);
balken(p1,p2,360);
bool[] auf0 = { false,false,false };
auflager(p0,auf0,0);
bool[] auf1 = { false,false,false };
auflager(p2,auf1,0);
bool[] ans10 = { false,false,false };
bool[] ans20 = { false,false,true };
anschluss(p1,p0,53.13010235415599, ans10, 0);
anschluss(p0,p1,53.13010235415599, ans20, 0);
bool[] ans11 = { false,false,false };
bool[] ans21 = { false,false,false };
anschluss(p2,p1,360, ans11, 0);
anschluss(p1,p2,360, ans21, 0);
