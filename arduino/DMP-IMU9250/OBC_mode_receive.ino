void OBC_mode_receive()
{
  /*
    Function to set the Bluepill to receiving mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: None, but gets mode from User Serial1 input
    OUTPUT: None, updates OBC_mode_value
  */

  if (Serial.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
        while (Serial.available() > 0)
        {
          bufferString += (char)Serial.read(); // Adds chars to the Serial1 buffer
        }
    // Conversion from String to int
    OBC_mode_value = bufferString.toInt();
    Serial.print("Mode Number: ");
    Serial.println(OBC_mode_value);
  }
}
