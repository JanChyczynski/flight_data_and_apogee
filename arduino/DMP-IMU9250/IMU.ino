void IMU() {
  fifoCount = mpu.getFIFOCount(); //Se obtiene el numero de bytes del DMP
  if (fifoCount >= 42) { //Si el numero de datos en el DMP es mayor a 42 bytes (mayor que el packetSize)
    mpu.resetFIFO(); //Se resetea el FIFO
    fifoCount = mpu.getFIFOCount(); //Se vuelven a obtener el numero de bytes en el FIFO del DMP
  }
  while (fifoCount < packetSize) { //Mientras el numero de datos es menor al paquete
    fifoCount = mpu.getFIFOCount(); //Se vuelven a obtener el número de bytes en el FIFO del DMP
    delay(1); //El delay es necesario para controlar el fifoCount. No se ha podido ver exactamente como afecta porque al crear un println el problema se resuelve por el incremento de tiempo.
    if (fifoCount > packetSize) { //Si el numero de datos en el DMP es mayor a 42 bytes (mayor que el packetSize)
      mpu.resetFIFO(); //Se resetea el FIFO
      fifoCount = mpu.getFIFOCount(); //Se vuelven a obtener el numero de bytes en el FIFO del DMP
    }
  }
  fifoCount = fifoCount - packetSize; //Se resta al fifoCount el packetSize (el resultado debería ser siempre 0)
  mpu.getFIFOBytes(fifoBuffer, packetSize); //Se obtienen los valores del DMP en forma de bytes
  mpu.dmpGetQuaternion(&q, fifoBuffer); //Se obtiene el cuaternion de actitud mediante el DMP
  mpu.dmpGetGyro(&v, fifoBuffer);
  
  vx = v.x;
  vy = v.y;
  vz = v.z; //vz is in [º/s]
  
  QuatToEuler(q, yaw, pitch, roll); //Se hace el paso del cuaternion de actitud a los ángulos de Euler
  
// The range of yaw, pitch adn roll anfles goes from 180 to -180: We need to adapt it to 0 to 360 range
  if (yaw < 0) yaw += 360;
  if (pitch < 0) pitch += 360;
  if (roll < 0) roll += 360;
// Variable standardization to the ones used in the original program (the range of these angles goes from 0 to 360)
  Yaw_deg=yaw;
  Pitch_deg=pitch;
  Roll_deg=roll;
}
