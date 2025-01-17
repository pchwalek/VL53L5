/*
   VL53L5cx class library implementation

   Copyright (c) 2021 Simon D. Levy

   MIT License
 */

#include "Lidar_VL53L5cx.h"

#include "vl53l5cx_plugin_motion_indicator.h"
#include "vl53l5cx_plugin_xtalk.h"

VL53L5cx::VL53L5cx(
				GPIO_TypeDef *lpnGPIOx,
				uint16_t lpnPin,
				I2C_HandleTypeDef *i2c_han,
        resolution_t resolution,
        target_order_t targetOrder,
        uint8_t rangingFrequency)
{
    _lpnGPIOx = lpnGPIOx;
    _lpnPin = lpnPin;

    _i2c_han = i2c_han;

    _resolution = resolution;
    _target_order = targetOrder;

    _ranging_frequency = rangingFrequency;

    // This can be changed in the begin() method
    _dev.platform.address = 0x52;

    _dev.platform.Write = BSP_I2C1_WriteReg16;
    _dev.platform.Read = BSP_I2C1_ReadReg16;
    _dev.platform.GetTick = BSP_GetTick;
}
        
void VL53L5cx::begin(uint8_t address)
{
    init(address);

    start_ranging();
}

void VL53L5cx::begin(detection_thresholds_t & values, uint8_t address)
{
    init(address);

    VL53L5CX_DetectionThresholds array[VL53L5CX_NB_THRESHOLDS] = {};
    make_detection_thresholds_array(values, array, _resolution);
    vl53l5cx_set_detection_thresholds(&_dev, array);
    vl53l5cx_set_ranging_frequency_hz(&_dev, 10);
    vl53l5cx_start_ranging(&_dev);

    start_ranging();
}

void VL53L5cx::make_detection_thresholds_array(
        detection_thresholds_t & values, 
        VL53L5CX_DetectionThresholds * array,
        resolution_t resolution)
{
    uint8_t size = resolution == RESOLUTION_8X8 ? 64 : 16;

    // Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8)
    for (uint8_t i = 0; i < size; i++) {

        // The first wanted thresholds is GREATER_THAN mode. Please note that
        // the first one must always be set with a mathematic_operation
        // VL53L5CX_OPERATION_NONE.  For this example, the signal thresholds is
        // set to 150 kcps/spads (the format is automatically updated inside
        // driver).
        array[2*i].zone_num = i;
        array[2*i].measurement = VL53L5CX_SIGNAL_PER_SPAD_KCPS;
        array[2*i].type = VL53L5CX_GREATER_THAN_MAX_CHECKER;
        array[2*i].mathematic_operation = VL53L5CX_OPERATION_NONE;
        array[2*i].param_low_thresh = values.kpcs_spads_min;
        array[2*i].param_high_thresh = values.kpcs_spads_max;

        // The second wanted checker is IN_WINDOW mode. We will set a
        // mathematical thresholds VL53L5CX_OPERATION_OR, to add the previous
        // checker to this one.  For this example, distance thresholds are set
        // between 200mm and 400mm (the format is automatically updated inside
        // driver).
        array[2*i+1].zone_num = i;
        array[2*i+1].measurement = VL53L5CX_DISTANCE_MM;
        array[2*i+1].type = VL53L5CX_IN_WINDOW;
        array[2*i+1].mathematic_operation = VL53L5CX_OPERATION_OR;
        array[2*i+1].param_low_thresh = values.distance_min;
        array[2*i+1].param_high_thresh = values.distance_max;
    }

    // The last thresholds must be clearly indicated. As we have 32
    // checkers (16 zones x 2), the last one is the 31
    uint8_t last = 2 * size - 1;
    array[last].zone_num = VL53L5CX_LAST_THRESHOLD | array[last].zone_num;
}

void VL53L5cx::init(uint8_t address)
{
    // Bozo filter for ranging frequency
	//todo : probably dont need the checks
    check_ranging_frequency(RESOLUTION_4X4, 60, "4X4");
    check_ranging_frequency(RESOLUTION_8X8, 15, "8X8");

    // Reset the sensor by toggling the LPN pin
    resetSensor();

//    vl53l5cx_init(&_dev);
//    HAL_Delay(1000);

    // Set the I^2C address
//    vl53l5cx_set_i2c_address(&_dev, address);
    _dev.platform.address = address;

    // Make sure there is a VL53L5CX sensor connected
    uint8_t isAlive = 0;
    uint8_t error = vl53l5cx_is_alive(&_dev, &isAlive);
    if(!isAlive || error) {
    	while(1);
//        Debugger::reportForever("VL53L5CX not detected at requested address");
    }

    // Init VL53L5CX sensor
    error = vl53l5cx_init(&_dev);
    if(error) {
//        Debugger::reportForever("VL53L5CX ULD Loading failed");
    }

    // Set resolution
    vl53l5cx_set_resolution(&_dev,
            _resolution == RESOLUTION_4X4 ?
            VL53L5CX_RESOLUTION_4X4 :
            VL53L5CX_RESOLUTION_8X8);

    // Set target order
    vl53l5cx_set_target_order(&_dev,
            _target_order == TARGET_ORDER_STRONGEST ?
            VL53L5CX_TARGET_ORDER_STRONGEST :
            VL53L5CX_TARGET_ORDER_CLOSEST);

} // init

void VL53L5cx::check_ranging_frequency(resolution_t resolution,
                                       uint8_t maxval,
                                       const char *label)
{
    if (_ranging_frequency < 1 || _ranging_frequency > maxval) {
//        Debugger::reportForever("Ranging frequency for %s resolution "
//                "must be at least 1 and no more than %d", maxval);
    }
}


void VL53L5cx::start_ranging(void)
{
    checkStatus(vl53l5cx_start_ranging(&_dev), "start error = 0x%02X");
}

bool VL53L5cx::isReady(void)
{
    uint8_t ready = false;
    vl53l5cx_check_data_ready(&_dev, &ready);
    return ready;
}

void VL53L5cx::collectData(void)
{
    vl53l5cx_get_ranging_data(&_dev, &_results);
}

uint8_t VL53L5cx::getStreamCount(void)
{
    return _dev.streamcount;
}

uint8_t VL53L5cx::getTargetStatus(uint8_t zone, uint8_t target)
{
    return _results.target_status[VL53L5CX_NB_TARGET_PER_ZONE * zone + target];
}

uint8_t VL53L5cx::getDistance(uint8_t zone, uint8_t target)
{
    return _results.distance_mm[VL53L5CX_NB_TARGET_PER_ZONE * zone + target];
}

uint8_t VL53L5cx::getSignalPerSpad(uint8_t zone, uint8_t target)
{
    return _results.signal_per_spad[VL53L5CX_NB_TARGET_PER_ZONE * zone + target];
}

#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
uint8_t VL53L5cx::getRangeSigma(uint8_t zone, uint8_t target)
{
    return _results.range_sigma_mm[VL53L5CX_NB_TARGET_PER_ZONE * zone + target];
}
#endif

uint8_t VL53L5cx::getNbTargetDetected(uint8_t zone)
{
    return _results.nb_target_detected[zone];
}

uint8_t VL53L5cx::getAmbientPerSpad(uint8_t zone)
{
    return _results.ambient_per_spad[zone];
}

#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
uint8_t VL53L5cx::getNbSpadsEnabled(uint8_t zone)
{
    return _results.nb_spads_enabled[zone];
}
#endif

void VL53L5cx::stop(void)
{
    vl53l5cx_stop_ranging(&_dev);
}

uint32_t VL53L5cx::getIntegrationTimeMsec(void)
{
    uint32_t integration_time_ms = 0;
    checkStatus(
            vl53l5cx_get_integration_time_ms(&_dev, &integration_time_ms),
            "vl53l5cx_get_integration_time_ms failed, status %u\n");
    return integration_time_ms;
}

void VL53L5cx::addMotionIndicator(uint16_t distanceMin, uint16_t distanceMax)
{
    // Create motion indicator
    VL53L5CX_Motion_Configuration motion_config = {};
    checkStatus(vl53l5cx_motion_indicator_init(&_dev, &motion_config, 
                _resolution == RESOLUTION_4X4 ?
                VL53L5CX_RESOLUTION_4X4 :
                VL53L5CX_RESOLUTION_8X8),
            "Motion indicator init failed with status : %u");

    if (distanceMin > 0 && distanceMax > 0) {

        bozoFilter(distanceMin < 400,
                "Motion indicator min distance must be at least 400mm");

        bozoFilter(distanceMax < distanceMin,
                "Motion indicator max distance must be greater than min");

        bozoFilter(distanceMax-distanceMin > 1500,
                "Motion indicator max distance can be no more than 1500mm above min");

        checkStatus(vl53l5cx_motion_indicator_set_distance_motion(
                        &_dev, &motion_config, distanceMin, distanceMax),
                "Motion indicator set distance failed with status : %u\n");
    }
}

void VL53L5cx::calibrateXtalk(
        uint8_t reflectancePercent,
        uint8_t samples,
        uint16_t distance)
{
    rangeFilter(reflectancePercent, 1, 99,  "Reflectance perecent");
    rangeFilter(samples, 1, 16, "Number of samples");
    rangeFilter(distance, 600, 3000, "Distance");

    checkStatus(vl53l5cx_calibrate_xtalk(&_dev, reflectancePercent, samples, distance),
            "vl53l5cx_calibrate_xtalk failed, status %u");
}

void VL53L5cx::getXtalkCalibrationData(VL53L5cx::XtalkCalibrationData & data)
{
    vl53l5cx_get_caldata_xtalk(&_dev, data.data);
}

void VL53L5cx::setXtalkCalibrationData(VL53L5cx::XtalkCalibrationData & data)
{
}

//VL53L5cxAutonomous::VL53L5cxAutonomous(
//        uint8_t lpnPin,
//        uint32_t integrationTimeMsec,
//        resolution_t resolution,
//        target_order_t targetOrder)
//    : VL53L5cxAutonomous::VL53L5cx(
//            lpnPin,
//            resolution,
//            targetOrder)
//{
//    _integration_time_msec = integrationTimeMsec;
//}

//void VL53L5cxAutonomous::begin(uint8_t address)
//{
//    init(address);
//
//    // Set ranging mode autonomous
//    checkStatus(vl53l5cx_set_ranging_mode(&_dev, VL53L5CX_RANGING_MODE_AUTONOMOUS),
//            "vl53l5cx_set_ranging_mode failed, status %u\n");
//
//    vl53l5cx_set_ranging_frequency_hz(&_dev, _ranging_frequency);
//
//    // Using autonomous mode, the integration time can be updated (not possible
//    // using continuous)
//    vl53l5cx_set_integration_time_ms(&_dev, _integration_time_msec);
//
//    start_ranging();
//}

void VL53L5cx::bozoFilter(bool cond, const char * msg)
{
    if (cond) {
//        Debugger::reportForever(msg);
    }
}

void VL53L5cx::checkStatus(uint8_t error, const char * fmt)
{
    if (error) {
//        Debugger::reportForever(fmt, error);
    }
}

void VL53L5cx::rangeFilter(
        uint16_t val,
        uint16_t minval,
        uint16_t maxval,
        const char * valname)
{
    if (val < minval || val > maxval) {
//        Debugger::reportForever("%s must be between %d and %d", valname, minval, maxval);
    }
}

void VL53L5cx::resetSensor(){ //todo: check if polarity is correct
	// Set pin LPN to LOW
	HAL_GPIO_WritePin(_lpnGPIOx, _lpnPin,
			GPIO_PIN_RESET);

	HAL_Delay(100);

	// Set pin LPN to HIGH
	HAL_GPIO_WritePin(_lpnGPIOx, _lpnPin,
			GPIO_PIN_SET);
}

//int32_t VL53L5cx::Write(uint16_t address, uint16_t regAddr, uint8_t *p_value, uint16_t bytes){
//	if (HAL_OK
//			== HAL_I2C_Mem_Write(_i2c_han, address, regAddr, 2, p_value, bytes, 10)) {
//		return 0;
//	} else {
//		return 1;
//	}
//}
//
//int32_t VL53L5cx::Read(uint16_t address, uint16_t regAddr, uint8_t *p_value, uint16_t bytes){
//	if (HAL_OK
//			== HAL_I2C_Mem_Read(_i2c_han, address, regAddr, 2, p_value, bytes, 10)) {
//		return 0;
//	} else {
//		return 1;
//	}
//}
//
//int32_t VL53L5cx::GetTick(void){
//	return HAL_GetTick();
//}
//
/**

  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write

  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP statu
  */
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  HAL_StatusTypeDef tst = HAL_I2C_Mem_Write(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT);

  if (tst != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF)
    {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  int32_t ret = BSP_ERROR_NONE;

  if (HAL_I2C_Mem_Read(&hi2c1, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, BUS_I2C1_POLL_TIMEOUT) != HAL_OK)
  {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
    {
      ret =  BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    }
    else
    {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

/**
  * @brief  Return system tick in ms
  * @retval Current HAL time base time stamp
  */
int32_t BSP_GetTick(void) {
  return HAL_GetTick();
}

