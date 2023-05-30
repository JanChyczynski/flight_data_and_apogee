void QuatToEuler(Quaternion q, float &yaw, float &pitch, float &roll) {
  //El proceso que seguidamente se muestra es completamente algebráico, en la memoria del trabajo se encuentra explicado el proceso.
  test = q.x * q.z - q.w * q.y;
  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;
  roll = (180 / pi) * atan2(2 * q.y * q.z + 2 * q.w * q.x, sqz - sqy - sqx + sqw);
  pitch = -(180 / pi) * asin(2 * test);
  yaw = (180 / pi) * atan2(2 * q.x * q.y + 2 * q.w * q.z, sqx + sqw - sqz - sqy);
  //Se asegura que en caso de gimbal lock, el resultado esté definido
  if (pitch >= 89.5) {
    pitch = 90;
    yaw = (180 / pi) * 2 * atan2(q.z, q.w);
    roll = 0;
  } 
  
  else if (pitch <= -89.5) {
    pitch = -90;
    yaw = (180 / pi) * 2 * atan2(q.z, q.w);
    roll = 0;
  }
}
