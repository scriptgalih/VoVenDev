import processing.serial.Serial; // serial library
import controlP5.*; // controlP5 library
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.sql.Timestamp;
import java.util.Date;
import java.text.SimpleDateFormat;
import java.awt.AWTException;
import java.awt.Robot;
//aaa
Robot robot; 

Serial serial;

ControlP5 cp5;

Chart graph_volume, graph_flow, graph_pressure;
PImage bgHome;
Button btn_set, btn_start, btn_stop;
Button btn_add_f, btn_add_ie, btn_min_f, btn_min_ie;

ListBox listCom;
Textlabel txtlblWhichcom; 
ListBox commListbox;
ListBox portlist;

PFont font;
//arialFont = loadFont("Arial", 32);

int posxcom = 850;
int posycom = -40;

int serial_conect = 0;
int commListMax;
int[] data = null;

color yellow_ = color(200, 200, 20), green_ = color(46, 209, 2), red_ = color(120, 30, 30), blue_ = color (0, 102, 200);
color white_ = color(255, 255, 255), black_ = color(0, 0, 0), pink_ = color(255, 0, 255), grey_ = color(#636e72);

//ControlFont cf1 = new ControlFont(createFont("Arial",28,true),28);

int SETFREQ=10, SETIE=3;
float DISPVOLUME=0, DISPOXYGEN=0, DISPFLOW=0;
float val_graph_vol=0, val_graph_flow=0, val_graph_pressure=0;
boolean firstload = true;

JSONObject json_set;

void setup() {
  //size(1024, 600);
  noCursor();
  fullScreen();
  font = createFont("Verdana", 10);
  cp5 = new ControlP5(this); 
  bgHome = loadImage("home.png");
  initJSON();
  setup_UART();
  buttonLayout();
  graphLayout();
  mainValue();
  //robot.mouseMove(0,0);
  firstload = false;
}

void draw() {
  background(bgHome);
  mainValue();
  serialStatus();
}
