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
void balken(pair z1, pair z2) {
  //die normale Linie
  path p = (z1 -- z2);
  draw(p,black + linewidth(0.5mm));
  //die gestrichelte
  //draw(shift()*p,black + linewidth(0.5mm));
}

pair p0 = (0,0);
pair p1 = (3,4);
pair p2 = (9,4);
balken(p0,p1);
balken(p1,p2);
bool[] auf0 = { false,false,false };
auflager(p0,auf0,0);
bool[] auf1 = { false,false,false };
auflager(p2,auf1,0);
