accelX = 0;
accelY = 0;
accelZ = 0;
gyroX = 0;
gyroY = 0;
gyroZ = 0;

for i = 1:32
accelX = accelX + Xdown.data(i, 1);
accelY = accelY + Xdown.data(i, 2);
accelZ = accelZ + Xdown.data(i, 3);
gyroX = gyroX + Xdown.data(i, 4);
gyroY = gyroY + Xdown.data(i, 5);
gyroZ = gyroZ + Xdown.data(i, 6);
end

accelX = accelX / 32;
accelY = accelY / 32;
accelZ = accelZ / 32;
gyroX = gyroX / 32;
gyroY = gyroY / 32;
gyroZ = gyroZ / 32;
