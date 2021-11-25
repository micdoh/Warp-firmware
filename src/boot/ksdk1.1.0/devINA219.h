void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus	concalSensorINA219(uint8_t REGISTER, uint16_t payload);
void        printSensorDataINA219(bool currentFlag, uint8_t REGISTER);