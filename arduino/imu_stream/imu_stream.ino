/*
 * imu_stream -- Arduino Nano 33 IoT as a USB IMU for the rover.
 *
 * Stand-in for the BMI088 until it arrives. The Nano 33 IoT carries an
 * LSM6DS3 (3-axis accel + 3-axis gyro) on the board itself.
 *
 * Board:   Arduino Nano 33 IoT
 * Library: "Arduino_LSM6DS3"  (Library Manager -> search LSM6DS3)
 *
 * Line format, one per sample, plain ASCII:
 *
 *     I,<micros>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
 *
 *     micros : Arduino's own clock, microseconds since boot (uint32,
 *              wraps every ~71 min -- the Python side unwraps it)
 *     ax..az : acceleration in g      (LSM6DS3 native unit)
 *     gx..gz : angular rate in deg/s  (LSM6DS3 native unit)
 *
 * We send the RAW sensor units and let Python convert to m/s^2 and
 * rad/s. That keeps the conversion in one place (config.py) where it
 * can be checked, instead of being baked into firmware we would have
 * to reflash to fix.
 *
 * The micros() stamp is the whole point of this format: it is taken
 * right after the sample is read, so it says when the IMU was actually
 * measured -- not when the packet happened to reach the PC. That is
 * what makes IMU/encoder time-sync calibration possible later.
 */

#include <Arduino_LSM6DS3.h>

// 115200 works but is the bottleneck at ~104 Hz. 500000 has plenty of
// headroom over USB CDC (where baud is nominal anyway).
const unsigned long SERIAL_BAUD = 500000;

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ;  // wait for the USB CDC port to be opened by the PC
  }

  if (!IMU.begin()) {
    // Say so forever rather than silently sending nothing.
    while (1) {
      Serial.println("E,IMU init failed");
      delay(1000);
    }
  }

  // One banner line so the PC can confirm what it is talking to and at
  // what rate. Starts with '#' so the parser skips it.
  Serial.print("#imu_stream LSM6DS3 accel_Hz=");
  Serial.print(IMU.accelerationSampleRate());
  Serial.print(" gyro_Hz=");
  Serial.println(IMU.gyroscopeSampleRate());
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  // Emit only when BOTH have fresh data. The reads are sequential;
  // micros() below timestamps this pair, not exact sensor synchronization.
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    unsigned long t = micros();

    // Build the line manually; String concatenation fragments the heap.
    Serial.print("I,");
    Serial.print(t);
    Serial.print(',');
    Serial.print(ax, 5); Serial.print(',');
    Serial.print(ay, 5); Serial.print(',');
    Serial.print(az, 5); Serial.print(',');
    Serial.print(gx, 4); Serial.print(',');
    Serial.print(gy, 4); Serial.print(',');
    Serial.println(gz, 4);
  }
}
