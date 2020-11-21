void sendSerial(String txt) {
  serial.write(txt);
}

void controlEvent(ControlEvent theControlEvent)
{

  if (theControlEvent.isController()) {
    if (firstload) {
      return;
    }
    //println(theControlEvent.getController().getName()); //check event name
    String controllerName = theControlEvent.getController().getName();
    if (controllerName=="portComList") {
      println("smile");
      InitSerial(theControlEvent.getController().getValue());
      return;
    }
    if (controllerName == "btn_add_f" ) {
      if (SETFREQ>=15)
        return;
      SETFREQ+=1;
    } else if (controllerName == "btn_min_f") {
      if (SETFREQ<=8)
        return;
      SETFREQ-=1;
    } else if (controllerName == "btn_add_ie") {
      if (SETIE>=5)
        return;
      SETIE+=1;
    } else if (controllerName == "btn_min_ie") {
      if (SETIE<=2)
        return;
      SETIE-=1;
    } else if (controllerName == "btn_set") {
      print(json_set.toString().replace("\n", "").replace(" ", "") + "\n");
      if (serial_conect==1)
        sendSerial(json_set.toString().replace("\n", "").replace(" ", "") + "\n");
    } else if (controllerName == "btn_start") {
      print("{\"c\":\"start\"}\n");
      if (serial_conect==1)
        sendSerial("{\"c\":\"start\"}\n");
    } else if (controllerName == "btn_stop") {
      print("{\"c\":\"stop\"}\n");
      if (serial_conect==1)
        sendSerial("{\"c\":\"stop\"}\n");
    }

    if (controllerName == "btn_add_f" || controllerName == "btn_min_f" ||controllerName == "btn_add_ie" ||controllerName == "btn_min_ie") {
      json_set.setInt("f", SETFREQ);
      json_set.setInt("ie", SETIE);
    }
    try {
      // These coordinates are screen coordinates
      int xCoord = 0;
      int yCoord = 0;

      // Move the cursor
      Robot robot = new Robot();
      robot.mouseMove(xCoord, yCoord);
    } 
    catch (AWTException e) {
    }
  }
}

void updateGraph() {
  graph_volume.push("val_volume", val_graph_vol);
  graph_flow.push("val_flow", val_graph_flow);
  graph_pressure.push("val_pressure", val_graph_pressure);
}
