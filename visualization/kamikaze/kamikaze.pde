import oscP5.*;
import netP5.*;

// osc connection
OscP5 oscP5;
NetAddress myRemoteLocation;

// sensor values
float ax, ay, az;
float ox,oy,oz;

// wekinator output values
float wek_up =0; 
float wek_idle =1;
int is_calibrated;

// define colors for visualization
color[][] c_performance = { {#de7920, #ba5e20} , // bad
                            {#f3ca6f, #d39b37} , // medium
                            {#6ab187,#20948b}    // good
                          };
color c_greylight = #f2f2f2;
color c_greymedium = #999999; 

// variables for text
PFont font_heading;
PFont font_text;
PFont font_counter;

// variables for svg
PShape svg_push_up;
PShape svg_push_down;

// variables for data
int counter =0;
int score;
int perf_value;
boolean has_started = false;
float countVertical;
float last_az;
float startOx, startOy, startOz;
float diffOx, diffOy, diffOz;
boolean is_up = true;
boolean is_down = false;

int performance = 2;
String[] performance_text = {"BAD", "MEDIUM", "GOOD"};

void setup() {
  size(1024, 768);
  pixelDensity(displayDensity());
  background(#e6e6e6);
  ellipseMode(CENTER);
  
  font_heading = loadFont("Roboto-Black-48.vlw");
  font_text = loadFont("Roboto-Regular-18.vlw");
  font_counter = loadFont("RobotoSlab-Bold-48.vlw");
  
  svg_push_up = loadShape("push-up.svg");
  svg_push_up.disableStyle();
  svg_push_up.setFill(#ff0000);
  
  svg_push_down = loadShape("push-down.svg");
  svg_push_down.disableStyle();
  svg_push_down.setFill(#ff0000);
  
  oscP5 = new OscP5(this,6648);

}


void draw() {

  background(#e6e6e6);
  noStroke();
  
  textAlign(CENTER);
  
  // write heading
  fill(c_greymedium);
  textFont(font_heading, 48);
  text("KAMIKAZE", width/2, 130);
  text("PUSH-UPS", width/2, 180);
  
  // draw outer circle
  switch(performance) {
    case 0:
      perf_value = 5;
      break;
    case 1:
       perf_value =10;
       break;
    case 2:
       perf_value =15;
       break;
  }
  
  
  fill(c_performance[performance][0]);
  ellipse(width/2, height/2, 270,270);
   
  // draw inner circle
  fill(c_performance[performance][1], 100);
  ellipse(width/2, height/2, 190,190);
  fill(c_performance[performance][1]);
  ellipse(width/2, height/2, 160,160);
  
  // write counter
  fill(#e6e6e6);
  textFont(font_counter, 48);
  text(counter, width/2, height/2);
  
  // draw svg
  if(is_up) {
    shape(svg_push_up, width/2-35, height/2);
  } else {
    shape(svg_push_down, width/2-35, height/2);
  }

  // write values
  textAlign(RIGHT);
  fill(c_greymedium);
  textFont(font_heading, 18);
  text("Performance:", width/2, height-150);
  text("Score:", width/2, height-125);
  textAlign(LEFT);
  textFont(font_text, 18);
  fill(c_performance[performance][0]);
  text(performance_text[performance], width/2+10, height-150);
  fill(c_greymedium);
  text(score, width/2+10, height-125);
  /*
  // write test values to screen
  text("ax: "+ax, 40, 40);
  text("ay: "+ay, 40, 65);
  text("az: "+az, 40, 90);
  text("ox: "+ox, 40, 115);
  text("oy: "+oy, 40, 140);
  text("oz: "+oz, 40, 165);
  text("countVertical: "+countVertical, 40, 190);
  text("is calibrated: "+is_calibrated, 40, 250);
  text("wekdata1: "+wek_up, 40, 290);
  text("wekdata2: "+wek_idle, 40, 310);
  */
}


void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  println("### received an osc message.");
  if (theOscMessage.addrPattern().equals("/data")) {
    ox = theOscMessage.get(0).floatValue();
    oy = theOscMessage.get(1).floatValue();
    oz = theOscMessage.get(2).floatValue();
    ax = theOscMessage.get(3).floatValue();
    ay = theOscMessage.get(4).floatValue();
    az = theOscMessage.get(5).floatValue();
  } else if (theOscMessage.addrPattern().equals("/calibrated")) {
    is_calibrated = theOscMessage.get(0).intValue();
  }
  
  // read wekinator values
  if (theOscMessage.addrPattern().equals("/output_1")) {
    wek_up = 1.0;
    is_up = false;
  }
  if (theOscMessage.addrPattern().equals("/output_2")) {
    wek_up = 0.0;
    wek_idle=1.0;
    is_up = true;
  } 
  //println(theOscMessage.addrPattern());
  //println("ACCELERATION: ax: "+ax+", ay: ",ay+", az: ",az);
  //println("ORIENTATION: ox: "+ox+", oy: "+oy+", oz: "+oz);
  //println("CALIBRATION: "+is_calibrated);
  
  /*
  // count push-ups without machine learning values
  if(has_started && abs(az-last_az) >0.01) {
    countVertical += (az*sq(0.1))/2;
    last_az = az;
    
    if(countVertical < -0.1) {
      is_down = true;
      is_up = false;
    }
    
    if (is_down == true && abs(countVertical) < 0.05) {
       counter++;
       is_down = false;
       is_up = true;
       score += perf_value;
       
     }
     */
     
     // calculate performance using the orientation values from the sensor
     diffOx = abs(startOx - ox);
     diffOy = abs(startOy - oy);
     diffOz = abs(startOz - oz);

     if (diffOx > 0.4 || diffOy > 0.4 || diffOz > 0.4 ) {
       performance =0;
     } else if (diffOx > 0.2 || diffOy > 0.2 || diffOz > 0.2 ) {
       performance =1;
     } else {
       performance =2;
     }
 // }

  // count push-ups WITH machine learning values
  if(wek_up == 1.0 && wek_idle==1.0) {
       counter++;
       score += perf_value;
       wek_idle =0.0;
  }

}

void keyPressed() {
  if (key == 's') {
    has_started = true;
    countVertical = 0.0;
    counter=0;
    score=0;
    startOx = ox;
    startOy = oy;
    startOz = oz;
  }
}