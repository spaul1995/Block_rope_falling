color c = color(0);
float x = 250;
float y = 0;
float radius = 10;
float g = 9.8;
float dt = 0.01;
float k = 100;
float b = 0.1;
float krope = 5;
float brope = 1;
float te = 0;
float initialsep = 400;
int n_rope =10;
//float theta_initial= (float)Math.PI/3;
float theta_initial= 0;
PVector[][] rcmrel = new PVector[2][4];
PVector[][] r = new PVector[2][4];
PVector[][] v = new PVector[2][4];
PVector[] ropepoints = new PVector[n_rope];
PVector[] forcerope = new PVector[n_rope];
float[] mass_rope = new float[n_rope];
PVector[] vr = new PVector[n_rope];
float ropesegleng=0;
PVector seperation = new PVector(initialsep,0);
{
r[0][0]=new PVector(225-initialsep/2,100);
r[0][1]=new PVector(275-initialsep/2,100);
r[0][2]=new PVector(275-initialsep/2,150);
r[0][3]=new PVector(225-initialsep/2,150);
r[1][0]=PVector.add(r[0][0], seperation);
r[1][1]=PVector.add(r[0][1], seperation);
r[1][2]=PVector.add(r[0][2], seperation);
r[1][3]=PVector.add(r[0][3], seperation);
v[0][0]=v[1][0]=new PVector(0,0);
v[0][1]=v[1][1]=new PVector(0,0);
v[0][2]=v[1][2]=new PVector(0,0);
v[0][3]=v[1][3]=new PVector(0,0);

}




PVector[] w = new PVector[2];
{
w[0]=new PVector(0,0);
w[1]=new PVector(0,0);
}
float[] theta = new float[2];
{
  theta[0]=theta_initial;
  theta[1]=theta_initial;
}

PVector[] xycm =new PVector[2];
{
xycm[0] = new PVector((r[0][1].x+r[0][2].x+r[0][3].x+r[0][0].x)/4,(r[0][1].y+r[0][2].y+r[0][3].y+r[0][0].y)/4);
xycm[1] = new PVector((r[1][1].x+r[1][2].x+r[1][3].x+r[1][0].x)/4,(r[1][1].y+r[1][2].y+r[1][3].y+r[1][0].y)/4);
}
{
  rcmrel[0][0]=PVector.sub(r[0][0],xycm[0]);
  rcmrel[0][1]=PVector.sub(r[0][1],xycm[0]);
  rcmrel[0][2]=PVector.sub(r[0][2],xycm[0]);
  rcmrel[0][3]=PVector.sub(r[0][3],xycm[0]);
  rcmrel[1][0]=PVector.sub(r[1][0],xycm[1]);
  rcmrel[1][1]=PVector.sub(r[1][1],xycm[1]);
  rcmrel[1][2]=PVector.sub(r[1][2],xycm[1]);
  rcmrel[1][3]=PVector.sub(r[1][3],xycm[1]);
}
PVector[] vcm = new PVector[2];
{
  vcm[0]=new PVector(0,0);
  vcm[1]=new PVector(0,0);
}
PVector[] force = new PVector[2];
{
  force[0]=new PVector(0,0);
  force[1]=new PVector(0,0);  
}
PVector[] torque = new PVector[2];
{
  torque[0]=new PVector(0,0);
  torque[1]=new PVector(0,0);  
}
float[] mass = new float[2];
{
  mass[0] = 1;
  mass[1] = 1;
}
float t = 0;
//float I = 200;
float[] I = new float[2];
{
  I[0] = 200;
  I[1] = 200;
}
Table table;

void setup() {
  pushMatrix();
  translate(xycm[0].x, xycm[0].y);
  for (int j=0;j<4;j++){
  rcmrel[0][j].rotate(theta[0]);
  }
  popMatrix();
  pushMatrix();
  translate(xycm[1].x, xycm[1].y);
  for (int j=0;j<4;j++){
  rcmrel[1][j].rotate(theta[1]);
  }
  popMatrix();
  for (int i=0;i<4;i++){
    r[0][i]=PVector.add(xycm[0],rcmrel[0][i]);
    v[0][i]=PVector.add(vcm[0],rcmrel[0][i].cross(w[0]));
    r[1][i]=PVector.add(xycm[1],rcmrel[1][i]);
    v[1][i]=PVector.add(vcm[1],rcmrel[1][i].cross(w[1]));
  }
  {
    for (int i=0;i<n_rope;i++){
      forcerope[i] = new PVector(0,0);
      vr[i] = new PVector(0,0);
      ropepoints[i]=PVector.add(r[0][0],PVector.div(PVector.sub(r[1][0],r[0][0]),float(n_rope+1)/float(i+1)));
      print(ropepoints[i]);
      mass_rope[i]=0.05;
    }
    ropesegleng=PVector.sub(ropepoints[0],ropepoints[1]).mag();
}
  size(500,500);
  table = new Table();
    table.addColumn("time");
  table.addColumn("te");
  table.addColumn("z0");
  table.addColumn("z1");
}

void draw() {
  background(255);
  checkcollision(0);
  checkcollision(1);
  move();
  saveFrame("output/task1_####.jpg");
}


void foreceropecalc(int ropenosindex) {
if (ropenosindex==0){
  if (PVector.sub(r[0][0],ropepoints[ropenosindex]).mag()>ropesegleng){
      PVector displacement = new PVector((r[0][0].x - ropepoints[ropenosindex].x), (r[0][0].y - ropepoints[ropenosindex].y));
      PVector unitdisplacement = new PVector((r[0][0].x - ropepoints[ropenosindex].x), (r[0][0].y - ropepoints[ropenosindex].y)).normalize();
      PVector damp_force = PVector.mult(unitdisplacement,(brope*(v[0][0].dot(unitdisplacement) - vr[ropenosindex].dot(unitdisplacement))));
      PVector spring_force = PVector.mult(unitdisplacement,(krope*(displacement.mag() - ropesegleng)));
      PVector netforce = PVector.add(damp_force,spring_force);
      forcerope[ropenosindex].add(netforce);
      torque[0].sub(PVector.sub(r[0][0],xycm[0]).cross(netforce));
      force[0].sub(netforce);     
  }
}
else if (ropenosindex==n_rope-1){
  if (PVector.sub(r[1][0],ropepoints[ropenosindex]).mag()>ropesegleng){
      PVector displacement = new PVector((r[1][0].x - ropepoints[ropenosindex].x), (r[1][0].y - ropepoints[ropenosindex].y));
      PVector unitdisplacement = new PVector((r[1][0].x - ropepoints[ropenosindex].x), (r[1][0].y - ropepoints[ropenosindex].y)).normalize();
      PVector damp_force = PVector.mult(unitdisplacement,(brope*(v[1][0].dot(unitdisplacement) - vr[ropenosindex].dot(unitdisplacement))));
      PVector spring_force = PVector.mult(unitdisplacement,(krope*(displacement.mag() - ropesegleng)));
      PVector netforce = PVector.add(damp_force,spring_force);
      forcerope[ropenosindex].add(netforce);
      torque[1].sub(PVector.sub(r[1][0],xycm[1]).cross(netforce));
      force[1].sub(netforce);     
  }
  if (PVector.sub(ropepoints[ropenosindex],ropepoints[ropenosindex-1]).mag()>ropesegleng){
      PVector displacement = new PVector((ropepoints[ropenosindex].x - ropepoints[ropenosindex-1].x), (ropepoints[ropenosindex].y - ropepoints[ropenosindex-1].y));
      PVector unitdisplacement = new PVector((ropepoints[ropenosindex].x - ropepoints[ropenosindex-1].x), (ropepoints[ropenosindex].y - ropepoints[ropenosindex-1].y)).normalize();
      PVector damp_force = PVector.mult(unitdisplacement,(brope*(vr[ropenosindex].dot(unitdisplacement) - vr[ropenosindex-1].dot(unitdisplacement))));
      PVector spring_force = PVector.mult(unitdisplacement,(krope*(displacement.mag() - ropesegleng)));
      PVector netforce = PVector.add(damp_force,spring_force);
      forcerope[ropenosindex-1].add(netforce);
      //torque[1].sub(PVector.sub(r[1][0],xycm[1]).cross(netforce));
      forcerope[ropenosindex].sub(netforce);     
  }   
  
}
else{
  if (PVector.sub(ropepoints[ropenosindex],ropepoints[ropenosindex-1]).mag()>ropesegleng){
      PVector displacement = new PVector((ropepoints[ropenosindex].x - ropepoints[ropenosindex-1].x), (ropepoints[ropenosindex].y - ropepoints[ropenosindex-1].y));
      PVector unitdisplacement = new PVector((ropepoints[ropenosindex].x - ropepoints[ropenosindex-1].x), (ropepoints[ropenosindex].y - ropepoints[ropenosindex-1].y)).normalize();
      PVector damp_force = PVector.mult(unitdisplacement,(brope*(vr[ropenosindex].dot(unitdisplacement) - vr[ropenosindex-1].dot(unitdisplacement))));
      PVector spring_force = PVector.mult(unitdisplacement,(krope*(displacement.mag() - ropesegleng)));
      PVector netforce = PVector.add(damp_force,spring_force);
      forcerope[ropenosindex-1].add(netforce);
      //torque[1].sub(PVector.sub(r[1][0],xycm[1]).cross(netforce));
      forcerope[ropenosindex].sub(netforce);     
  }   
}
}

void move() {
  vcm[0].y=vcm[0].y+(g+(force[0].y/mass[0]))*dt;
  vcm[0].x=vcm[0].x+((force[0].x/mass[0]))*dt;
  force[0] = new PVector(0,0);
  xycm[0].x=xycm[0].x+vcm[0].x*dt;
  xycm[0].y=xycm[0].y+vcm[0].y*dt;
  w[0].add(PVector.mult(torque[0],(dt/I[0])));
  //print(w[0]);
  theta[0]+=w[0].z*dt;
  
  vcm[1].y=vcm[1].y+(g+(force[1].y/mass[1]))*dt;
  vcm[1].x=vcm[1].x+((force[1].x/mass[1]))*dt;
  force[1] = new PVector(0,0);
  xycm[1].x=xycm[1].x+vcm[1].x*dt;
  xycm[1].y=xycm[1].y+vcm[1].y*dt;
  w[1].add(PVector.mult(torque[1],(dt/I[1])));
  //print(w[1]);
  theta[1]+=w[1].z*dt;
  
  for (int i=0;i<n_rope;i++){
     foreceropecalc(i);
  }
  
   for (int i=0;i<n_rope;i++){
      vr[i].add(PVector.mult(PVector.add(new PVector(0,g),PVector.div(forcerope[i],mass_rope[i])),dt));
      ropepoints[i].add(PVector.mult(vr[i],dt));
      forcerope[i]=new PVector(0,0);
      //print(ropepoints[i]);
   }

  te=0.5*mass[0]*(vcm[0].x*vcm[0].x+vcm[0].y*vcm[0].y)+0.5*I[0]*w[0].z*w[0].z+mass[0]*g*(height-xycm[0].y)+0.5*mass[1]*(vcm[1].x*vcm[1].x+vcm[1].y*vcm[1].y)+0.5*I[1]*w[1].z*w[1].z+mass[1]*g*(height-xycm[1].y);
   for (int i=0;i<n_rope;i++){
     te+=0.5*mass_rope[i]*(vr[i].x*vr[i].x+vr[i].y*vr[i].y)+mass_rope[i]*g*(height-ropepoints[i].y);
   }
  TableRow newRow = table.addRow();
  newRow.setFloat("time", t);
  newRow.setFloat("te", te);
  newRow.setFloat("z0", height-xycm[0].y);
  newRow.setFloat("z1", height-xycm[1].y);
  saveTable(table, "data/new.csv");
  t+=dt;
  rotater();
  for (int i=0;i<4;i++){
    r[0][i]=PVector.add(xycm[0],rcmrel[0][i]);
    v[0][i]=PVector.add(vcm[0],rcmrel[0][i].cross(w[0]));
    r[1][i]=PVector.add(xycm[1],rcmrel[1][i]);
    v[1][i]=PVector.add(vcm[1],rcmrel[1][i].cross(w[1]));
  }
  
}


void rotater() {
  pushMatrix();
  translate(xycm[0].x, xycm[0].y);
  for (int j=0;j<4;j++){
  rcmrel[0][j].rotate(w[0].z*dt);
  }
  rotate(theta[0]);
  display(0);
  popMatrix();
  
  pushMatrix();
  translate(xycm[1].x, xycm[1].y);
  for (int j=0;j<4;j++){
  rcmrel[1][j].rotate(w[1].z*dt);
  }
  rotate(theta[1]);
  display(1);
  popMatrix();
  
  ellipse(r[0][1].x,r[0][1].y,radius,radius);
  ellipse(r[0][2].x,r[0][2].y,radius,radius);
  ellipse(r[0][3].x,r[0][3].y,radius,radius);
  

  ellipse(r[1][1].x,r[1][1].y,radius,radius);
  ellipse(r[1][2].x,r[1][2].y,radius,radius);
  ellipse(r[1][3].x,r[1][3].y,radius,radius);
  
  fill(color(64,255,0));
  ellipse(r[0][0].x,r[0][0].y,radius,radius);
  ellipse(r[1][0].x,r[1][0].y,radius,radius);
  for (int i=0;i<n_rope;i++){
    ellipse(ropepoints[i].x,ropepoints[i].y,radius,radius);
  }
  
}


void checkcollision(int index){
  PVector[] forcer = new PVector[4];
  {
    forcer[0] = new PVector(0,0);
    forcer[1] = new PVector(0,0);
    forcer[2] = new PVector(0,0);
    forcer[3] = new PVector(0,0);
    
  }
  torque[index]=new PVector(0,0,0);
  for (int i=0;i<4;i++){
    if (r[index][i].y>height){
    forcer[i].y=-k*(r[index][i].y-height)-b*v[index][i].y;
    torque[index].add(PVector.sub(r[index][i],xycm[index]).cross(forcer[i]));
    force[index].y+=forcer[i].y;
    print("collision!!    ");
    }
  }
   PVector[] frope = new PVector[n_rope];
  {
   for (int i=0;i<n_rope;i++){
     frope[i] = new PVector(0,0);
   }
  } 
  for (int i=0;i<n_rope;i++){
    if (ropepoints[i].y>height-radius){
    frope[i].y=-k/100*(ropepoints[i].y-height+radius)-b*vr[i].y;
    forcerope[i].y+=frope[i].y;
    }
  }
  }


void display(int counter) {
  fill(c);
  //print(xycm[counter], "    ");
  rectMode(CENTER);
  rect(0,0,50,50);
}
