#include "dj_app.hpp"

// 여기서부터 Run 함수까지는 roboclaw, pwm_out 함수를 참고하여 만든 코드라서 나도 정확히 이해하지 못함. 대강 이런 기능이겠구나 추측은 됨

RS485::RS485() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::ttyS3)
{

}

RS485::~RS485()
{
	close(_rs485_fd);
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int RS485::task_spawn(int argc, char *argv[])
{
	RS485 *instance = new RS485();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int RS485::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RS485::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("dj_app", "example");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int RS485::print_status()
{
	_mixing_output.printStatus();
	return 0;
}

void RS485::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	if (!_rs485_initialized) {
		initializeRS485();
		_rs485_initialized = true;
	}

	if (!_drivers_initialized) {
		initializeDrivers();
		_drivers_initialized = true;
	}

	_mixing_output.update();

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	_actuator_armed_sub.update();
	_mixing_output.updateSubscriptions(true);

	if (_vehicle_control_mode_sub.updated()) {
		vehicle_control_mode_s vehicle_control_mode{};

		if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
			_manual_driving = vehicle_control_mode.flag_control_manual_enabled;
			_mission_driving = vehicle_control_mode.flag_control_auto_enabled;
		}
	}

	perf_end(_cycle_perf);
}

// outputs[0] : 오른바퀴 출력
// outputs[1] : 왼바퀴 출력
bool RS485::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{


	if (_manual_driving == true)	// Manual Mode인 경우
	{
		// Dead Zone 기능 추가
		if (outputs[0] >= 30 && outputs[0] <= 70) {stop_motors = true;}
    		if (outputs[1] >= 30 && outputs[1] <= 70) {stop_motors = true;}
		// 후진, 제자리 회전을 위해 오프셋 추가
		outputs[0] -= 50;
		outputs[1] -= 50;
 	}

	else if (_mission_driving == true)	// Mission Mode인 경우
	{
		// 한 쪽만 속도가 0인 경우 -> 회전하려는 것이므로 반대쪽의 속도에 -를 곱한 값을 복사해서 제자리 회전하게 한다.
		if (outputs[0] == 0 && outputs[1] != 0) outputs[0] = -outputs[1];
		if (outputs[1] == 0 && outputs[0] != 0) outputs[1] = -outputs[0];
	}

	else if (_mission_driving != true) {stop_motors = true;}	// Manual도 Mission도 아닌 경우 모터 정지

	if (stop_motors)
	{
		// setRpm(&_rtu_left, &_driver_left, 0);
		// setRpm(&_rtu_right, &_driver_right, 0);

		// setRpm(&_rtu_broad, &_driver_broad, 0);

		setRpmWToq(&_rtu_left, &_driver_left, 0, 0);
		setRpmWToq(&_rtu_right, &_driver_right, 0, 0);

		// setRpmWToq(&_rtu_broad, &_driver_broad, 0, 0);
	}
	else
	{
		// setRpm(&_rtu_left, &_driver_left, (int16_t)outputs[1]);
		// setRpm(&_rtu_right, &_driver_right, -(int16_t)outputs[0]);

		setRpmWToq(&_rtu_left, &_driver_left, (int16_t)outputs[1], 0);
		setRpmWToq(&_rtu_right, &_driver_right, -(int16_t)outputs[0], 0);
	}
	return true;
}

void RS485::initializeDrivers()
{
	setMode(&_rtu_left, &_driver_left, Mode::TORQUE);
	setMode(&_rtu_right, &_driver_right, Mode::TORQUE);
	enableMotor(&_rtu_left, &_driver_left);
	enableMotor(&_rtu_right, &_driver_right);
}

double RS485::rpmToRadPerSec(double rpm)
{
	return rpm*2*PI/60.0;
}

double RS485::rpmToLinear(double rpm)
{
	double w_wheel = rpmToRadPerSec(rpm);
	double v = w_wheel*_r_wheel;
	return v;
}

void RS485::setMode(RTU* rtu, DriverState* driver, Mode mode)
{
	driver->mode = mode;
	if (driver->node_id == 0x00) {_driver_left.mode = mode; _driver_right.mode = mode;}
	writeSingleRegisterUsingUsleep(rtu, rtu->_node_id, RegisterAddr::OPR_MODE, (uint8_t*)&mode, sizeof(mode));
}

Mode RS485::getMode(RTU* rtu)
{
	readRegisters(rtu, RegisterAddr::OPR_MODE, 1);
	Mode mode = static_cast<Mode>((_read_buf_address[0] << 8) | _read_buf_address[1]);
	return mode;
}

void RS485::enableMotor(RTU *rtu, DriverState* driver)
{
	driver->enabled = true;
	if (driver->node_id == 0x00) {_driver_left.enabled = true; _driver_right.enabled = true;}
	uint16_t enable = RegisterAddr::ENABLE;
	writeSingleRegisterUsingUsleep(rtu, rtu->_node_id, RegisterAddr::CONTROL_REG, (uint8_t*)&enable, sizeof(enable));

}

void RS485::disableMotor(RTU *rtu, DriverState* driver)
{
	driver->enabled = false;
	uint16_t disable = RegisterAddr::UNDEFINED;
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::CONTROL_REG, (uint8_t*)&disable, sizeof(disable));
}

void RS485::emergencyStopMotor(RTU *rtu, DriverState* driver)
{
	driver->enabled = false;
	uint16_t emergency_stop = RegisterAddr::EMER_STOP;
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::CONTROL_REG, (uint8_t*)&emergency_stop, sizeof(emergency_stop));
}

void RS485::clearAlarm(RTU *rtu)
{
	uint16_t alarm = RegisterAddr::ALRM_CLR;
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::CONTROL_REG, (uint8_t*)&alarm, sizeof(alarm));
}

void RS485::setRpm(RTU* rtu, DriverState* driver, int16_t cmd_rpm)
{
	// 모터드라이버가 속도 모드가 아니면 속도 모드로 설정
	if (driver->mode != Mode::VELOCITY) setMode(rtu, driver, Mode::VELOCITY);
	// 모터드라이버가 enable 상태가 아니면 enable 설정
	if (driver->enabled != true) enableMotor(rtu, driver);

	driver->cmd_rpm = cmd_rpm;

	int16_t rpm_arr[2];
	rpm_arr[0] = cmd_rpm; rpm_arr[1] = cmd_rpm;
	writeRegisters(rtu->_node_id, RegisterAddr::L_CMD_RPM, (uint8_t*)rpm_arr, sizeof(rpm_arr));
}

void RS485::getRpm(RTU* rtu, DriverState* driver)
{
	readRegisters(rtu, RegisterAddr::L_FB_RPM, 2);
	driver->L_rpm = (int16_t)((_read_buf_address[0] << 8) | _read_buf_address[1]) / 10.0;
	driver->R_rpm = (int16_t)((_read_buf_address[2] << 8) | _read_buf_address[3]) / 10.0;
}

void RS485::getLinearVelocities(RTU* rtu, DriverState* driver)
{
	getRpm(rtu, driver);
	driver->L_vel = rpmToLinear(driver->L_rpm);
	driver->R_vel = rpmToLinear(driver->R_rpm);
}

void RS485::setTorque(RTU* rtu, DriverState* driver, int16_t toq)
{
	// 모터드라이버가 토크 모드가 아니면 토크 모드로 설정
	if (driver->mode != Mode::TORQUE) setMode(rtu, driver, Mode::TORQUE);
	// 모터드라이버가 enable 상태가 아니면 enable 설정
	if (driver->enabled != true) enableMotor(rtu, driver);

	int16_t toq_arr[2];
	toq_arr[0] = toq; toq_arr[1] = toq;
	writeRegisters(rtu->_node_id, RegisterAddr::L_CMD_TOQ, (uint8_t*)toq_arr, sizeof(toq_arr));
}

void RS485::getTorque(RTU* rtu, DriverState* driver)
{
	readRegisters(rtu, RegisterAddr::L_FB_TOQ, 2);
	driver->L_toq = (int16_t)((_read_buf_address[0] << 8) | _read_buf_address[1]) / 10.0;
	driver->R_toq = (int16_t)((_read_buf_address[2] << 8) | _read_buf_address[3]) / 10.0;
}

void RS485::setMaxRpm(RTU* rtu, uint16_t max_rpm)
{
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::MOTOR_MAX_RPM, (uint8_t*)&max_rpm, sizeof(max_rpm));
}


// EDIT 좌우 마진을 다르게 줘보고 싶어서 수정함.
// 이전 버전으로 돌릴려면 margin 지우고 함수 안에 값들 상수로 넣어줘
void RS485::setRpmWToq(RTU* rtu, DriverState* driver, int16_t CMD_Rpm, int16_t margin)
{
	if (CMD_Rpm == driver->cmd_rpm) return;
	else driver->cmd_rpm = CMD_Rpm;

	int16_t toq = PARAM::RATED_TORQUE;

	getRpm(rtu, driver);

	if (driver->cmd_rpm == 0){
		if (driver->R_rpm > 0){
			setMaxRpm(rtu,1);
			setTorque(rtu, driver, -toq);
			while (driver->R_rpm > driver->cmd_rpm + margin){
				getRpm(rtu, driver);
			}
			setTorque(rtu, driver, 0);
		}

		else if (driver->R_rpm < 0){
			setMaxRpm(rtu, 1);
			setTorque(rtu, driver, toq);
			while (driver->R_rpm < driver->cmd_rpm - margin){
				getRpm(rtu, driver);
			}
			setTorque(rtu, driver, 0);
		}

		else{
			setMaxRpm(rtu, 1);
			setTorque(rtu, driver, 0);
		}
	}

	else if (driver->cmd_rpm * (driver->R_rpm) > 0){
		toq = (driver->cmd_rpm > 0) ? PARAM::RATED_TORQUE : -PARAM::RATED_TORQUE;
		if (abs(driver->cmd_rpm) >= abs(driver->R_rpm)){
			setMaxRpm(rtu, abs(driver->cmd_rpm));
			setTorque(rtu, driver, toq);
		}

		else{
			setMaxRpm(rtu, abs(driver->cmd_rpm));
			setTorque(rtu, driver, -toq);
			while (abs(driver->R_rpm) >= (abs(driver->cmd_rpm) + margin)){
				getRpm(rtu, driver);
			}
			setTorque(rtu, driver, toq);
		}

		// else{
		// 	setMaxRpm(rtu, abs(driver->cmd_rpm));
		// 	setTorque(rtu, driver, toq);
		// } 역토크 안 주고 자연 정지 테스트용
	}

	else if (driver->cmd_rpm * (driver->R_rpm) <= 0){
		toq = (driver->cmd_rpm > 0) ? PARAM::RATED_TORQUE : -PARAM::RATED_TORQUE;
		setMaxRpm(rtu, abs(driver->cmd_rpm));
		setTorque(rtu, driver, toq);
	}
}

void RS485::setMaxLCurrent(RTU* rtu, uint16_t max_cur)
{
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::L_MAX_CUR, (uint8_t*)&max_cur, sizeof(max_cur));
}

void RS485::setMaxRCurrent(RTU* rtu, uint16_t max_cur)
{
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::R_MAX_CUR, (uint8_t*)&max_cur, sizeof(max_cur));
}

void RS485::setRatedLCurrent(RTU* rtu, uint16_t rated_cur)
{
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::L_RATED_CUR, (uint8_t*)&rated_cur, sizeof(rated_cur));
}

void RS485::setRatedRCurrent(RTU* rtu, uint16_t rated_cur)
{
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::R_RATED_CUR, (uint8_t*)&rated_cur, sizeof(rated_cur));
}

void RS485::getVoltage(RTU* rtu, DriverState* driver)
{
	readRegisters(rtu, RegisterAddr::DRIVER_VOL, 1);
	double voltage = ((_read_buf_address[0] << 8) | _read_buf_address[1]) / 100.0;
	driver->voltage = voltage;
}

void RS485::getDriverTemp(RTU* rtu, DriverState* driver)
{
	readRegisters(rtu, RegisterAddr::DRIVER_TEMP, 1);
	double temperature = (int16_t)((_read_buf_address[0] << 8) | _read_buf_address[1]) / 10.0;
	driver->temperature = temperature;
}

void RS485::setNodeID(RTU* rtu, uint16_t address)
{
	// 주소 바꾸는 패킷 전송
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::RS485_NODE_ID, (uint8_t*)&address, sizeof(address));

	// EEPROM에 저장하는 패킷 전송
	uint16_t EEPROM_value = 0x0001;
	writeSingleRegister(rtu, rtu->_node_id, RegisterAddr::SAVE_EEPROM, (uint8_t*)&EEPROM_value, sizeof(EEPROM_value));
}

// 시리얼 통신 파라미터 설정은 termios.h 을 통해서 한다.
// termios 구조체를 선언한 다음, 각종 파라미터들과의 비트연산으로 설정을 키거나 끈다.
// 그리고 설정을 마친 다음 tcsetattr() 함수로 디스크립터(_rs485_fd)에 설정 내용을 저장하는 방식인 듯 하다.
ssize_t RS485::initializeRS485()
{
	static constexpr int TIMEOUT_US = 11_ms;
	_rs485_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };

	// _rs485_fd = open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);	// rs485 디스크립터를 연다.
	_rs485_fd = open(_port_name, O_RDWR | O_NOCTTY );			// rs485 디스크립터를 연다.
	if (_rs485_fd < 0) return ERROR;

	ssize_t ret = 0;
	struct termios rs485_config {};			// termios 구조체 초기화
	ret = tcgetattr(_rs485_fd, &rs485_config);	// 현재 터미널 설정 termios 구조체에 가져옴
	if (ret < 0) return ERROR;

	rs485_config.c_cflag &= ~PARENB;	// Parity bit None
	rs485_config.c_cflag &= ~CSTOPB;	// Stop bit 1로 설정
	rs485_config.c_cflag &= ~CSIZE;		// Data bits 설정 초기화
	rs485_config.c_cflag |= CS8;		// Data bits 8로 설정
	rs485_config.c_cflag &= ~CRTSCTS;	// Flow Conotrol 끄기
	rs485_config.c_cflag |= CREAD | CLOCAL;

	cfsetispeed(&rs485_config, B115200);	// 입력 보드레이트
	cfsetospeed(&rs485_config, B115200);	// 출력 보드레이트
	// rs485_config.c_cc[VTIME] = 0;
	// rs485_config.c_cc[VMIN] = 0; 	// read 할 때 최소로 읽어올 character 수

	tcflush(_rs485_fd, TCIFLUSH);		// 디스크립터 초기화
	ret = tcsetattr(_rs485_fd, TCSANOW, &rs485_config);	// termios 구조체 설정을 디스크립터에 저장
	if (ret < 0) return ERROR;

	FD_ZERO(&_rs485_fd_set);
	FD_SET(_rs485_fd, &_rs485_fd_set);

	return OK;
}

void RS485::setRTUPacket(RTU *rtu, uint8_t device_address, uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length)
{
	rtu->_node_id = device_address;
	rtu->_function_code = function_code;
	rtu->_register_address_high = register_address >> 8;
	rtu->_register_address_low = register_address & 0xFF;
	for (size_t i = 0; i < data_length; i++)
		rtu->_data[i] = data[data_length - 1 - i];
}

uint16_t RS485::calculateCRC(uint8_t *data, size_t data_length)
{
	const uint16_t polynomial = 0xA001;
	uint16_t crc = 0xFFFF;

	for (size_t i = 0; i < data_length; ++i)
	{
		crc ^= data[i];
		for (int j = 0; j < 8; ++j)
        	{
			uint16_t LSB = crc & 0x0001;
			crc >>= 1;
			if (LSB) crc ^= polynomial;
        	}
	}
	uint8_t crc_index = (data_length > 6) ? 11 : 6;		// 데이터 길이에 따라 CRC 값을 쓸 index를 바꾼다.
	data[crc_index] = crc & 0xFF;
	data[crc_index + 1] = crc >> 8;
	return crc;
}

void RS485::writeSingleRegister(RTU *rtu, uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length)
{
	setRTUPacket(rtu, device_address, 0x06, register_address, data, data_length);	// CRC를 제외한 RTU 패킷 설정
	calculateCRC((uint8_t*)rtu, sizeof(*rtu) - 2);					// CRC 계산하여 RTU 패킷 완성
	write(_rs485_fd, rtu, sizeof(*rtu));						// RTU 패킷 전송

	// usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	static constexpr int TIMEOUT_US = 11_ms;
	_rs485_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };
	int select_status = select(_rs485_fd + 1, &_rs485_fd_set, nullptr, nullptr, &_rs485_fd_timeout);
	if (select_status <= 0) return;

	// function code 0x06으로 write를 해도 모터드라이버에서 응답을 하여 버퍼에 쌓이므로, 이를 제거한다.
	uint8_t buf[8];
	read(_rs485_fd, &buf, sizeof(buf));
}

void RS485::writeSingleRegisterUsingUsleep(RTU *rtu, uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length)
{
	setRTUPacket(rtu, device_address, 0x06, register_address, data, data_length);	// CRC를 제외한 RTU 패킷 설정
	calculateCRC((uint8_t*)rtu, sizeof(*rtu) - 2);					// CRC 계산하여 RTU 패킷 완성
	write(_rs485_fd, rtu, sizeof(*rtu));						// RTU 패킷 전송

	usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	// function code 0x06으로 write를 해도 모터드라이버에서 응답을 하여 버퍼에 쌓이므로, 이를 제거한다.
	uint8_t buf[8];
	read(_rs485_fd, &buf, sizeof(buf));
}

// 해당 함수는 현재 2개의 레지스터에만 값을 쓸 수 있다.(3개 이상 불가능)
void RS485::writeRegisters(uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length)
{
	// 기존에 만들어둔 RTU 패킷이 function Code 0x10의 패킷과는 맞지 않으므로, 함수 내에서 임시 패킷을 만들어 전송한다.
	// calculateCRC 위의 부분이 writeSingleRegister의 setRTUPacket 함수와 같은 기능을 하는 부분이고, 이하로는 writeSingleRegister와 동일하다.
	uint8_t packet[13];
	packet[0] = device_address;
	packet[1] = 0x10;
	packet[2] = register_address >> 8;
	packet[3] = register_address & 0xFF;
	packet[4] = 0x00; packet[5] = 0x02; packet[6] = 0x04;
	for (size_t i = 0; i < data_length; i++)
		packet[i + 7] = data[data_length - 1 - i];
	calculateCRC(packet, sizeof(packet) - 2);
	write(_rs485_fd, packet, sizeof(packet));

	// usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	static constexpr int TIMEOUT_US = 11_ms;
	_rs485_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };
	int select_status = select(_rs485_fd + 1, &_rs485_fd_set, nullptr, nullptr, &_rs485_fd_timeout);
	if (select_status <= 0) return;

	// function code 0x10으로 write를 해도 모터드라이버에서 응답을 하여 버퍼에 쌓이므로, 이를 제거한다.
	uint8_t buf[8];
	read(_rs485_fd, &buf, sizeof(buf));
}

void RS485::readRegisters(RTU* rtu, uint16_t register_address, uint16_t register_number)
{
	setRTUPacket(rtu, rtu->_node_id, 0x03, register_address, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)rtu, sizeof(*rtu) - 2);
	write(_rs485_fd, rtu, sizeof(*rtu));

	// usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	static constexpr int TIMEOUT_US = 11_ms;
	_rs485_fd_timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };
	int select_status = select(_rs485_fd + 2, &_rs485_fd_set, nullptr, nullptr, &_rs485_fd_timeout);
	if (select_status <= 0) return;

	static uint8_t buf[9];					// 수신 값을 저장할 버퍼(2개를 읽으면 9바이트가 들어오므로 배열의 길이를 9로 설정함.)
	ssize_t ret = read(_rs485_fd, &buf, sizeof(buf));	// read 함수로 읽는다.(ret에는 읽은 바이트 수가 저장됨.)
	if (ret <= 0) return;
	_read_buf_address = buf + 3;				// buf+3에 해당하는 주소는 우리가 원하는 데이터가 있는 주소이다.
}

extern "C" __EXPORT int dj_app_main(int argc, char *argv[])
{
	return RS485::main(argc, argv);

	// RS485 rs485;
	// rs485.initializeRS485();
	// rs485.setMode(&rs485._rtu_broad, &rs485._driver_broad, Mode::TORQUE);
	// Mode mode_left = rs485.getMode(&rs485._rtu_left);
	// Mode mode_right = rs485.getMode(&rs485._rtu_right);
	// PX4_INFO("%d  %d\n", mode_left, mode_right);

	// return 0;
}
