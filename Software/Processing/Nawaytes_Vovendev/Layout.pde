void buttonLayout() {
  noStroke();
  btn_start = cp5.addButton("btn_start")
    .setValue(0)
    .setPosition(277, 513)
    .setSize(220, 65)
    ;
  btn_start.getColor().setBackground(color(#993B85, 8));
  btn_start.getColor().setActive(color(#006633,75));
  btn_start.setCaptionLabel(" ");

  btn_stop = cp5.addButton("btn_stop")
    .setValue(0)
    .setPosition(527, 513)
    .setSize(220, 65)
    ;
  btn_stop.getColor().setBackground(color(#993B85, 8));
  btn_stop.getColor().setActive(color(#006633,75));
  btn_stop.setCaptionLabel(" ");
  
  btn_set = cp5.addButton("btn_set")
    .setValue(0)
    .setPosition(780, 397)
    .setSize(220, 65)
    ;
  btn_set.getColor().setBackground(color(#993B85, 8));
  btn_set.getColor().setActive(color(#006633,75));
  btn_set.setCaptionLabel(" ");
  
  btn_add_f = cp5.addButton("btn_add_f")
    .setValue(0)
    .setPosition(944, 167)
    .setSize(55, 77)
    ;
  btn_add_f.getColor().setBackground(color(#993B85, 8));
  btn_add_f.getColor().setActive(color(#006633,75));
  btn_add_f.setCaptionLabel(" ");
  
  btn_add_ie = cp5.addButton("btn_add_ie")
    .setValue(0)
    .setPosition(944, 300)
    .setSize(55, 77)
    ;
  btn_add_ie.getColor().setBackground(color(#993B85, 8));
  btn_add_ie.getColor().setActive(color(#006633,75));
  btn_add_ie.setCaptionLabel(" ");
  
  btn_min_f = cp5.addButton("btn_min_f")
    .setValue(0)
    .setPosition(779, 167)
    .setSize(55, 77)
    ;
  btn_min_f.getColor().setBackground(color(#993B85, 8));
  btn_min_f.getColor().setActive(color(#006633,75));
  btn_min_f.setCaptionLabel(" ");
  
  btn_min_ie = cp5.addButton("btn_min_ie")
    .setValue(0)
    .setPosition(779, 300)
    .setSize(55, 77)
    ;
  btn_min_ie.getColor().setBackground(color(#993B85, 8));
  btn_min_ie.getColor().setActive(color(#006633,75));
  btn_min_ie.setCaptionLabel(" ");
}

void graphLayout() {
  graph_volume = cp5.addChart("chart_volume")
    .setPosition(185, 102)
    .setSize(570, 130)
    .setRange(0, 600)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    ;
  graph_volume.getColor().setBackground(color(#191F27));
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
  graph_pressure.getColor().setBackground(color(#191F27));
  graph_pressure.setCaptionLabel(" ");

  graph_pressure.addDataSet("val_pressure");
  graph_pressure.setColors("val_pressure", color(yellow_), color(yellow_));
  graph_pressure.updateData("val_pressure", new float[1000]);
  
  graph_flow = cp5.addChart("graph_flow")
    .setPosition(185, 367)
    .setSize(570, 130)
    .setRange(-20, 40)
    .setView(Chart.LINE) // use Chart.LINE, Chart.PIE, Chart.AREA, Chart.BAR_CENTERED
    ;
  graph_flow.getColor().setBackground(color(#191F27));
  graph_flow.setCaptionLabel(" ");

  graph_flow.addDataSet("val_flow");
  graph_flow.setColors("val_flow", color(yellow_), color(yellow_));
  graph_flow.updateData("val_flow", new float[1000]);
}

void mainValue(){
  textSize(75);
  fill(255,255,255);
  textAlign(CENTER);
  text(int(DISPVOLUME),92,203); //Volume
  text(int(DISPOXYGEN),92,335); //Oxygen
  text(int(DISPFLOW),92,467); //FLow
  
  textSize(40);
  text(SETFREQ,890,221); //SET FREQUENCY
  text("1:"+SETIE,890,353); //SET IE
  
}
