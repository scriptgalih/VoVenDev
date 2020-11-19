void initJSON(){
 json_set = new JSONObject();
 
 json_set.setString("c","set");
 json_set.setInt("f",SETFREQ);
 json_set.setInt("ie",SETIE);
 //String command_set = json_set.toString();
 //command_set = command_set.replace("\n","");
 print(json_set.toString().replace("\n","").replace(" ",""));
}
