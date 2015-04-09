  // Need G4P library
import g4p_controls.*;

import processing.serial.*;
import java.util.Date;
import java.text.*;

PrintWriter log;

Serial myPort;  // Create object from Serial class
String buffer;     // Data received from the serial port
SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd_HHmm");

int lf = 10;      // ASCII linefeed 

String lastLogRow = "";

boolean connected = false;
boolean pauseUpdate = false;
JSONObject displayData = null;
boolean displayUpdate = false;
boolean hold = false;
boolean reverse = false;

public void setup(){
  size(480, 320, JAVA2D);
  createGUI();
  customGUI();
  // Place your setup code here

  // On Mac port name will be /dev/tty.usbmodem1421 or /dev/tty.usbmodem1411
  String portName = "/dev/tty.usbmodem1d114111";//Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil(lf);  
}

public void draw(){
  background(230);
  if (displayUpdate && !pauseUpdate){
    String val = Integer.toString(displayData.getInt("camcount"));
    textfieldCounter.setText(val);

    val = Float.toString(displayData.getFloat("distance"));
    textfieldDistance.setText(val);

    val = Float.toString(displayData.getFloat("speed"));
    textfieldSpeed.setText(val);

    val = Float.toString(displayData.getFloat("longitude"));
    textfieldLongitude.setText(val);

    val = Float.toString(displayData.getFloat("latitude"));
    textfieldLatitude.setText(val);

    val = displayData.getString("datetime");
    textfieldDateTime.setText(val);

    val = Integer.toString(displayData.getInt("period"));
    textfieldPeriod.setText(val);

    reverse = displayData.getInt("reverse") != 0 ? true : false;
    if (reverse){
       buttonReverse.setLocalColorScheme(GCScheme.RED_SCHEME);
    } else {
       buttonReverse.setLocalColorScheme(GCScheme.GREEN_SCHEME);
    }
    
    hold = displayData.getInt("hold") != 0 ? true : false;
    if (hold){
       buttonHold.setLocalColorScheme(GCScheme.RED_SCHEME);
    } else {
       buttonHold.setLocalColorScheme(GCScheme.GREEN_SCHEME);
    }
    
    displayUpdate = false;
  }
}

void serialEvent( Serial port) {
  if (0 < port.available()){
    //put the incoming data into a String - 
    //the '\n' is our end delimiter indicating the end of a complete packet
    //println("serialEvent");
    buffer = port.readStringUntil(lf);
    //make sure our data isn't empty before continuing
    if (buffer != null) {
      //trim whitespace and formatting characters (like carriage return)
     buffer = trim(buffer);
     //println(buffer);
      
      //look for our '&&LOG=' string to look for data to log
      if (buffer.indexOf("&CONNECT") > -1) {
        connect();
        return;
      }
      
      if (connected){
        //look for our '&&LOG=' string to look for data to log
        if (buffer.indexOf("&LOG=") > -1) {
          lastLogRow = buffer.substring(5);
          log.println(lastLogRow);
          log.flush();
          return;
        }        
      }

      if (connected){
        //look for json formated string to update display
        if (buffer.indexOf("[{") > -1) {
          println("parse json");
          JSONArray jsonArray = JSONArray.parse(buffer);
          println(jsonArray.toString());
          displayData = jsonArray.getJSONObject(0);
          displayUpdate = true;
          return;
        }        
      }
    }
    if (buffer.length() > 1){
      textareaLog.appendText(buffer + "\r\n");
      java.awt.Toolkit.getDefaultToolkit().beep();
    }
  }
}

boolean setValue(String name, String value){
  myPort.write("&SET="+name+":"+value+"\r\n");
  return true;
}

void capture(){
  myPort.write("&CAPTURE\r\n");
}

void connect(){
  connected = true;

  // Create a new file in the sketch directory
  Date now = new Date();
  String fileName = "balk_"+sdf.format(now)+".csv";
  log = createWriter(fileName);
}

void disconnect(){
  log.flush();
  log.close();
  connected = false;
}

// Use this method to add additional statements
// to customise the GUI controls
public void customGUI(){

}
