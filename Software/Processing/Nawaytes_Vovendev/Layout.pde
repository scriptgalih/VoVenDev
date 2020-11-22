void textfieldLayout() {
  noStroke();
  txt_name = cp5.addTextfield("txt_name")
  .setFont(font)
    .setPosition(195, 121)
    .setSize(486, 38)
    .setFocus(true)
    .setColor(color(#FFFFFF))
    
    ;
  txt_name.getColor().setBackground(color(#000000, 0));
  txt_name.setLabel(" ");
  
  txt_age = cp5.addTextfield("txt_age")
  .setFont(font)
    .setPosition(195, 221)
    .setSize(486, 38)
    .setFocus(true)
    .setColor(color(#FFFFFF))
    
    ;
  txt_age.getColor().setBackground(color(#000000, 0));
  txt_age.setLabel(" ");
}

void buttonLayout() {
  noStroke();
  btn_start = cp5.addButton("btn_start")
    .setValue(0)
    .setPosition(277, 513)
    .setSize(220, 65)
    ;
  btn_start.getColor().setBackground(color(#993B85, 8));
  btn_start.getColor().setActive(color(#006633, 75));
  btn_start.setCaptionLabel(" ");

  btn_stop = cp5.addButton("btn_stop")
    .setValue(0)
    .setPosition(527, 513)
    .setSize(220, 65)
    ;
  btn_stop.getColor().setBackground(color(#993B85, 8));
  btn_stop.getColor().setActive(color(#006633, 75));
  btn_stop.setCaptionLabel(" ");

  btn_set = cp5.addButton("btn_set")
    .setValue(0)
    .setPosition(780, 397)
    .setSize(220, 65)
    ;
  btn_set.getColor().setBackground(color(#993B85, 8));
  btn_set.getColor().setActive(color(#006633, 75));
  btn_set.setCaptionLabel(" ");

  btn_add_f = cp5.addButton("btn_add_f")
    .setValue(0)
    .setPosition(944, 167)
    .setSize(55, 77)
    ;
  btn_add_f.getColor().setBackground(color(#993B85, 8));
  btn_add_f.getColor().setActive(color(#006633, 75));
  btn_add_f.setCaptionLabel(" ");

  btn_add_ie = cp5.addButton("btn_add_ie")
    .setValue(0)
    .setPosition(944, 300)
    .setSize(55, 77)
    ;
  btn_add_ie.getColor().setBackground(color(#993B85, 8));
  btn_add_ie.getColor().setActive(color(#006633, 75));
  btn_add_ie.setCaptionLabel(" ");

  btn_min_f = cp5.addButton("btn_min_f")
    .setValue(0)
    .setPosition(779, 167)
    .setSize(55, 77)
    ;
  btn_min_f.getColor().setBackground(color(#993B85, 8));
  btn_min_f.getColor().setActive(color(#006633, 75));
  btn_min_f.setCaptionLabel(" ");

  btn_min_ie = cp5.addButton("btn_min_ie")
    .setValue(0)
    .setPosition(779, 300)
    .setSize(55, 77)
    ;
  btn_min_ie.getColor().setBackground(color(#993B85, 8));
  btn_min_ie.getColor().setActive(color(#006633, 75));
  btn_min_ie.setCaptionLabel(" ");
}

void graphLayout() {
  graph_volume = cp5.addChart("chart_volume")
    .setPosition(185, 102)
    .setSize(570, 130)
    .setRange(0, 600)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    ;
  graph_volume.getColor().setBackground(color(#191F27, 0));
  graph_volume.setCaptionLabel(" ");

  graph_volume.addDataSet("val_volume");
  graph_volume.setColors("val_volume", color(yellow_), color(yellow_));
  graph_volume.updateData("val_volume", new float[1000]);


  graph_pressure = cp5.addChart("chart_pressure")
    .setPosition(185, 235)
    .setSize(570, 130)
    .setRange(0, 60)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    ;
  graph_pressure.getColor().setBackground(color(#191F27, 0));
  graph_pressure.setCaptionLabel(" ");

  graph_pressure.addDataSet("val_pressure");
  graph_pressure.setColors("val_pressure", color(yellow_), color(yellow_));
  graph_pressure.updateData("val_pressure", new float[1000]);

  graph_flow = cp5.addChart("chart_flow")
    .setPosition(185, 367)
    .setSize(570, 130)
    .setRange(-20, 40)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    ;
  graph_flow.getColor().setBackground(color(#191F27, 0));
  graph_flow.setCaptionLabel(" ");

  graph_flow.addDataSet("val_flow");
  graph_flow.setColors("val_flow", color(yellow_), color(yellow_));
  graph_flow.updateData("val_flow", new float[1000]);
}

void mainValue() {
  textSize(75);
  fill(255, 255, 255);
  textAlign(CENTER);
  text(int(DISPVOLUME), 92, 203); //Volume
  text(int(DISPPRESSUSE), 92, 335); //Oxygen
  text(int(DISPOXYGEN), 92, 467); //FLow

  textSize(40);
  text(SETFREQ, 890, 221); //SET FREQUENCY
  text("1:"+SETIE, 890, 353); //SET IE
}

void supText() {
  textSize(20);
  fill(255, 255, 255);
  textAlign(RIGHT);
  int possuptexty = 120;
  int possuptextx = 750;
  text("Volume(mL)", possuptextx, possuptexty); //Volume
  text("Pressure(cmH2O)", possuptextx, possuptexty + 132); //Oxygen
  text("Flow(L/min)", possuptextx, possuptexty + (132*2)); //FLow

  textSize(15);
  fill(255, 255, 255);
  textAlign(LEFT);
  int possuptextxgtop = 190;
  int possuptextygtop = 118;
  int possuptextxgbot = 190;
  int possuptextygbot = 229;
  text(600, possuptextxgtop, possuptextygtop); //Volume
  text(60, possuptextxgtop, possuptextygtop + 132); //Oxygen
  text(40, possuptextxgtop, possuptextygtop + (132*2)); //FLow

  text(0, possuptextxgbot, possuptextygbot); //Volume
  text(0, possuptextxgbot, possuptextygbot + 132); //Oxygen
  text(-20, possuptextxgbot, possuptextygbot + (132*2)); //FLow
}

void setupTab() {
  cp5.addTab("info")
    ;
  cp5.getTab("default")
    .activateEvent(true)
    .setLabel("Home")
    .setId(1)
    .setHeight(40)
    .setSize(100, 40)
    .setColorBackground(color(#993B85, 8))
    .setColorLabel(color(255))
    ;
  cp5.getTab("info")
    .activateEvent(true)
    .setLabel("info")
    .setId(1)
    .setHeight(40)
    .setSize(100, 40)
    .setColorBackground(color(#993B85, 8))
    .setColorLabel(color(255))
    ;

  cp5.getTab("info")
    .activateEvent(true)
    .setLabel("Info")
    .setId(2)
    .setHeight(40)
    .setSize(100, 40)
    .setColorBackground(color(#993B85, 8))
    .setColorLabel(color(255))
    ;

  cp5.getController("chart_volume").moveTo("default");
  cp5.getController("chart_pressure").moveTo("default");
  cp5.getController("chart_flow").moveTo("default");

  cp5.getController("btn_start").moveTo("default");
  cp5.getController("btn_set").moveTo("default");
  cp5.getController("btn_stop").moveTo("default");

  cp5.getController("btn_add_f").moveTo("default");
  cp5.getController("btn_min_f").moveTo("default");
  cp5.getController("btn_add_ie").moveTo("default");
  cp5.getController("btn_min_ie").moveTo("default");

  cp5.getController("txt_name").moveTo("info");
  cp5.getController("txt_age" ).moveTo("info");
}
