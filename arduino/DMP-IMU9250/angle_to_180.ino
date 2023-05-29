float angle_to_180(float inputAngle){
  /*
   * input:any angle expressed in degrees
   * output:returns the same angle in degrees expressed between -179.99 and +180.00
   */
  
  while (inputAngle>180 || inputAngle<=-180)
  {
    if (inputAngle>180)
    {
      inputAngle-=360;
    }
    else if (inputAngle<=-180)
    {
      inputAngle+=360;
    }    
  }
  return inputAngle;
}
