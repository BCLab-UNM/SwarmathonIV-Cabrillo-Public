#include "Calibration.h"
#include <EEPROM.h>
#include <CRC32.h>

struct cal_t calibration;

bool stored_calibration_used = false;

void clearCalibration() {
  for (unsigned int i=0; i<sizeof(cal_t); i++) {
    EEPROM.write(i, 0xff);
  }
}

void clearObservedCalibration() {
  calibration.data.x_min = INT16_MAX;
  calibration.data.x_max = INT16_MIN;
  calibration.data.y_min = INT16_MAX;
  calibration.data.y_max = INT16_MIN;
  calibration.data.z_min = INT16_MAX;
  calibration.data.z_max = INT16_MIN;
}

void commitCalibration() {
  CRC32 gen;

  // Generate the crc32
  calibration.crc32 = gen.calculate(&calibration.data, sizeof(cal_t::cal_data_t));

  // Write EERPOM data
  for (unsigned int i=0; i<sizeof(cal_t); i++) {
    EEPROM.write(i, ((uint8_t *) &calibration)[i]);
  }
}

bool readCalibration(struct cal_t &cal) {
  CRC32 gen;

  // Read the EEPROM data
  for (unsigned int i=0; i<sizeof(cal_t); i++) {
    ((uint8_t *) &cal)[i] = EEPROM.read(i);
  }

  // Validate the header
  if (cal.header != CAL_HEADER_WORD)
    return false;

  // Validate the signature
  uint32_t expected_sig = gen.calculate(&cal.data, sizeof(cal_t::cal_data_t));
  if (cal.crc32 != expected_sig)
    return false;

  return true;
}
