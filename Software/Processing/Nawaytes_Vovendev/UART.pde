String shortifyPortName(String portName, int maxlen)  
{
  String shortName = portName;
  if (shortName.startsWith("/dev/")) shortName = shortName.substring(5);  
  if (shortName.startsWith("tty.")) shortName = shortName.substring(4); // get rid off leading tty. part of device name
  if (portName.length()>maxlen) shortName = shortName.substring(0, (maxlen-1)/2) + "~" +shortName.substring(shortName.length()-(maxlen-(maxlen-1)/2));
  if (shortName.startsWith("cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
}

void setup_UART()
{ 
  //Comport List Selection                   
  //listCom = cp5.addListBox("portComList", posxcom+5, posycom+85, 150, 240); // make a listbox and populate it with the available comm ports
  listCom = cp5.addListBox("portComList"); // make a listbox and populate it with the available comm ports
  listCom.setPosition(posxcom+5, posycom+85);
  listCom.setCaptionLabel("PORT COM");
  listCom.setColorBackground(black_);
  listCom.setFont(font);
  listCom.setSize(150, 240);
  listCom.setItemHeight(35)
    .setBarHeight(35);
  listCom.close();

  for (int i=0; i<Serial.list ().length; i++) 
  {

    String pn = shortifyPortName(Serial.list()[i], 13);
    if (pn.length() >0 ) listCom.addItem(pn, i); // addItem(name,value)
    commListMax = i;
  }

  listCom.addItem("Close Comm", ++commListMax); // addItem(name,value)
  // text label for which comm port selected
  txtlblWhichcom = cp5.addTextlabel("txtlblWhichcom", "No Port Selected", posxcom+5, posycom+62); // textlabel(name,text,x,y)
}

void InitSerial(float portValue) 
{
  if (portValue < commListMax) {
    String portPos = Serial.list()[int(portValue)];
    txtlblWhichcom.setValue("Connected = " + shortifyPortName(portPos, 8));
    serial = new Serial(this, portPos, 115200);
    serial.bufferUntil('\n');
    serial_conect=1;
    fill(0, 255, 0);
    ellipse(posxcom+130, posycom+67, 10, 10);
    noFill();
  } else 
  {
    txtlblWhichcom.setValue("Not Connected");
    serial.clear();
    serial.stop();
    serial_conect=0;
  }
}

void serialEvent (Serial usbPort) 
{
  try {
    String usbString = usbPort.readStringUntil ('\n');
    if (usbString == null)
      return;
      
      println(usbString); //--> for debuging

    float data[] = float(split(usbString, ','));
    //for (int sensorNum = 1; sensorNum < data.length; sensorNum++) { println(sensorNum + " " + data[sensorNum]);  } //--> for debuging
    /*
    Format paket
     HEADER#DATA1#DATA2#DATA3 ...
     24 = data grafik
     24#PRESSURE#FLOW#VOLUME
     */
    //PRESSURE_=data[1];
    //FLOW_=data[2];
    //VOLUME_=data[0];
    // exhale_time=data[4];
    
    if (int(data[0]) == 59)
    {
      //print(data[3]);
      //println("\t OK masuk");
      val_graph_vol= data[1];
      val_graph_flow= data[2];
      val_graph_pressure= data[3];
      //  //FLOW_ = data[2];
      //  //VOLUME_ = data[3];
      updateGraph();
    }
    //println(VOLUME_);
  }
  catch(RuntimeException e)
  {
    println("Serial event is not null");
    println(e);
  }
}

void serialStatus() {
  if (serial_conect == 1) {
    fill(0, 255, 0);
    ellipse(posxcom+130, posycom+67, 10, 10);
    noFill();
  } else {

    fill(128, 0, 0);
    ellipse(posxcom+130, posycom+67, 10, 10);
    noFill();
  }
}
