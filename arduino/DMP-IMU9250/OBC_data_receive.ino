void OBC_data_receive()
{
  /*
    Function to set the Bluepill to receiving data mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: None, but gets data from User Serial1 input
    OUTPUT: Save OBC_data_value
  */
  //IMU();
  // Check if UART Serial1 is available
  if (Serial.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial.available() > 0)
    {
      bufferString += (char)Serial.read(); // adds chars to the Serial1 buffer
    }
    OBC_data_value = bufferString.toInt(); // Conversion from String to int
  }
}
