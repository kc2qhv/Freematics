/*************************************************************************
* OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreematicsONE.h>
#include "config.h"
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20

static uint8_t lastFileSize = 0;
static uint16_t fileIndex = 0;

uint16_t MMDD = 0;
uint32_t UTC = 0;

#if USE_MPU6050 || USE_MPU9250
byte accCount = 0; // count of accelerometer readings
long accSum[3] = {0}; // sum of accelerometer data
int accCal[3] = {0}; // calibrated reference accelerometer data
long gyrSum[3] = {0}; // sum of accelerometer data
int gyrCal[3] = {0}; // calibrated reference accelerometer data
long magSum[3] = {0}; // sum of accelerometer data
int magCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp; // device temperature (celcius degree)
#endif

class ONE : public COBDSPI, public CDataLogger
#if USE_MPU6050
,public CMPU6050
#elif USE_MPU9250
,public CMPU9250
#endif
{
public:
    ONE():state(0) {}
    
    void setup()
    {
        state = 0;

        delay(750);
        byte ver = begin();
        Serial.print("FWV ");
        Serial.println(ver);

#if USE_MPU6050 || USE_MPU9250
        Wire.begin();
        Serial.print("MEMS ");
        if (memsInit()) {
          state |= STATE_MEMS_READY;
          Serial.println(PW);
        } else {
          Serial.println(NW);
        }
#endif

#if ENABLE_DATA_LOG
        Serial.print("SD ");
        uint16_t volsize = initSD();
        if (volsize) {
          Serial.print(volsize);
          Serial.print("MB ");
          Serial.println(PW);
        } else {
          Serial.println(NW);
        }
#endif

        Serial.print("OBD ");
        if (init()) {
          Serial.println(PW);
        } else {
          Serial.println(NW);
          reconnect();
        }
        state |= STATE_OBD_READY;

#if GET_VIN
        // retrieve VIN
        char buffer[128];
        if ((state & STATE_OBD_READY) && getVIN(buffer, sizeof(buffer))) {
          // Serial.print("VIN ");
          // Serial.println(buffer);
          logData(PID_VIN, buffer);
        }
#endif

#if USE_GPS
        Serial.print("GPS ");
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_FOUND;
          Serial.println(PW);
        } else {
          Serial.println(NW);
        }
#endif

#if USE_MPU6050 || USE_MPU9250
        calibrateMEMS();
#endif
    }
    
#if USE_GPS
    void logGPSData()
    {
#if LOG_GPS_NMEA_DATA
        // issue the command to get NMEA data (one line per request)
        char buf[255];
        byte n = getGPSRawData(buf, sizeof(buf));
        if (n) {
            dataTime = millis();
            // strip heading $ and ending \r\n
            logData(buf + 1, n - 3);
        }
#endif

#if LOG_GPS_PARSED_DATA
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        
        if (getGPSData(&gd)) {
            dataTime = millis();
            if (gd.time && gd.time != UTC) {
              logData(PID_GPS_DATE, gd.date);
              logData(PID_GPS_TIME, gd.time);
              logData(PID_GPS_LATITUDE, gd.lat);
              logData(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              MMDD = gd.date;
              UTC = gd.time;
              state |= STATE_GPS_READY;
            }
        }
#endif
    }
#endif

#if ENABLE_DATA_LOG
    uint16_t initSD()
    {
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);

        Sd2Card card;
        uint32_t volumesize = 0;
        if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
            SdVolume volume;
            if (volume.init(card)) {
              volumesize = volume.blocksPerCluster();
              volumesize >>= 1; // 512 bytes per block
              volumesize *= volume.clusterCount();
              volumesize /= 1000;
            }
        }
        if (SD.begin(SD_CS_PIN)) {
          state |= STATE_SD_READY;
          return volumesize; 
        } else {
          return 0;
        }
    }
    
    void flushData()
    {
      // flush SD data every 1KB
        byte dataSizeKB = dataSize >> 10;
        if (dataSizeKB != lastFileSize) {
            flushFile();
            lastFileSize = dataSizeKB;
#if MAX_LOG_FILE_SIZE
            if (dataSize >= 1024L * MAX_LOG_FILE_SIZE) {
              closeFile();
              state &= ~STATE_FILE_READY;
            }
#endif
        }
    }
#endif

    void reconnect()
    {
        Serial.println("OBD RECONN");
        // try to re-connect to OBD
        if (init()) return;
#if ENABLE_DATA_LOG
        closeFile();
#endif

        // turn off GPS power
        initGPS(0);
        state &= ~(STATE_OBD_READY | STATE_GPS_READY);
        Serial.println("OBD STANDBY");
        // put OBD chips into low power mode
        enterLowPowerMode();
        
        // calibrate MEMS for several seconds
        for (;;) {
#if USE_MPU6050 || USE_MPU9250
          accSum[0] = 0;
          accSum[1] = 0;
          accSum[2] = 0;
          for (accCount = 0; accCount < 10; ) {
            readMEMS();
            sleepms(30);
          }
          // calculate relative movement
          unsigned long motion = 0;
          for (byte i = 0; i < 3; i++) {
            long n = accSum[i] / accCount - accCal[i];
            motion += n * n;
          }
          // check movement
          if (motion > START_MOTION_THRESHOLD) {
            
            Serial.print("MTN ");
            Serial.println(motion);
            // try OBD reading
#endif
            leaveLowPowerMode();
            if (init()) {
              // OBD is accessible
              break;
            }
            enterLowPowerMode();
#if USE_MPU6050 || USE_MPU9250
            // calibrate MEMS again in case the device posture changed
            calibrateMEMS();
          }
#endif
        }
#ifdef ARDUINO_ARCH_AVR
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
#else
        setup();
#endif
    }
    
#if USE_MPU6050 || USE_MPU9250
    void calibrateMEMS()
    {
        // get accelerometer calibration reference data
        accCal[0] = accSum[0] / accCount;
        accCal[1] = accSum[1] / accCount;
        accCal[2] = accSum[2] / accCount;
        gyrCal[0] = gyrSum[0] / accCount;
        gyrCal[1] = gyrSum[1] / accCount;
        gyrCal[2] = gyrSum[2] / accCount;
        magCal[0] = magSum[0] / accCount;
        magCal[1] = magSum[1] / accCount;
        magCal[2] = magSum[2] / accCount;
    }
    
    void readMEMS()
    {
        // load accelerometer and temperature data
        int16_t acc[3] = {0};
        int16_t gyr[3] = {0};
        int16_t mag[3] = {0};
        int16_t temp; // device temperature (in 0.1 celcius degree)
        memsRead(acc, gyr, mag, &temp);
        if (accCount >= 250) {
          accSum[0] >>= 1;
          accSum[1] >>= 1;
          accSum[2] >>= 1;
          gyrSum[0] >>= 1;
          gyrSum[1] >>= 1;
          gyrSum[2] >>= 1;
          magSum[0] >>= 1;
          magSum[1] >>= 1;
          magSum[2] >>= 1;
          accCount >>= 1;
        }
        accSum[0] += acc[0];
        accSum[1] += acc[1];
        accSum[2] += acc[2];
        gyrSum[0] += gyr[0];
        gyrSum[1] += gyr[1];
        gyrSum[2] += gyr[2];
        magSum[0] += mag[0];
        magSum[1] += mag[1];
        magSum[2] += mag[2];
        accCount++;
        deviceTemp = temp / 10;
    }
    
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        readMEMS();
      }
      delay(10);
    }
 #endif
    byte state;
};

static ONE one;

void setup()
{
    one.initSender();
    one.setup();
}

void loop()
{
#if ENABLE_DATA_LOG
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
      Serial.print("Fil ");
      int index = one.openFile(MMDD); // if GPS is not installed, always '0' and thus index-only; else DDMMYY-index
      if (index != 0) {
        one.state |= STATE_FILE_READY;
        Serial.println(index);
      } else {
        Serial.println("Err");
      }
    }
#endif
    if (one.state & STATE_OBD_READY) {
        byte pids[]= {0, PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
        byte pids2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_DISTANCE, PID_ENGINE_OIL_TEMP, PID_ENGINE_FUEL_RATE};
        int values[sizeof(pids)];
        static byte index2 = 0;
        pids[0] = pids2[index2 = (index2 + 1) % sizeof(pids2)];
        // read multiple OBD-II PIDs
        if (one.readPID(pids, sizeof(pids), values) == sizeof(pids)) {
          one.dataTime = millis();
          for (byte n = 0; n < sizeof(pids); n++) {
            one.logData((uint16_t)pids[n] | 0x100, values[n]);
          }
        }
        if (one.errors >= 10) {
            one.reconnect();
        }
    } else {
      if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
      }
    }
    
    // log battery voltage (from voltmeter), data in 0.01v
    int v = one.getVoltage() * 100;
    one.dataTime = millis();
    one.logData(PID_BATTERY_VOLTAGE, v);

#if USE_MPU6050 || USE_MPU9250
    if ((one.state & STATE_MEMS_READY) && accCount) {
       // log the loaded MEMS data
      one.logData(PID_ACC, accSum[0] / accCount / ACC_DATA_RATIO, accSum[1] / accCount / ACC_DATA_RATIO, accSum[2] / accCount / ACC_DATA_RATIO);
      one.logData(PID_GYRO, gyrSum[0] / accCount / GYRO_DATA_RATIO, gyrSum[1] / accCount / GYRO_DATA_RATIO, gyrSum[2] / accCount / GYRO_DATA_RATIO);
      one.logData(PID_COMPASS, magSum[0] / accCount / COMPASS_DATA_RATIO, magSum[1] / accCount / COMPASS_DATA_RATIO, magSum[2] / accCount / COMPASS_DATA_RATIO);
    }
#endif
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
      one.logGPSData();
    }
#endif

#if ENABLE_DATA_LOG
    uint32_t logsize = sdfile.size();
    if (logsize > 0) {
      one.flushData();
      Serial.print("SDF ");
      Serial.print(sdfile.size());
      Serial.println("B");
    }
#endif
}
