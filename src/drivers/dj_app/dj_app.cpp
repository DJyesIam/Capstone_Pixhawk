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

	_mixing_output.update();

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
	}

	_actuator_armed_sub.update();
	_mixing_output.updateSubscriptions(true);

	readRegisters(&_rtu_front, 0x2001, 0x0001);

	perf_end(_cycle_perf);
}

// front 0 : 앞 왼바퀴
// front 1 : 앞 오른바퀴
// back 0 : 뒤 왼바퀴
// back 1 : 뒤 오른바퀴
// outputs[0] : 오른바퀴 출력
// outputs[1] : 왼바퀴 출력
bool RS485::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (stop_motors)
	{
		setMotorSpeed(&_rtu_front, 0, 0);
		setMotorSpeed(&_rtu_front, 0, 1);
		setMotorSpeed(&_rtu_back, 0, 0);
		setMotorSpeed(&_rtu_back, 0, 1);
	}
	else
	{
		setMotorSpeed(&_rtu_front, outputs[1], 0);
		setMotorSpeed(&_rtu_front, -outputs[0], 1);
		setMotorSpeed(&_rtu_back, outputs[1], 0);
		setMotorSpeed(&_rtu_back, -outputs[0], 1);
	}
	return true;
}

void RS485::setMotorSpeed(RTU* rtu, uint16_t rpm, bool side)
{
	// 모터드라이버가 속도 모드가 아니면 속도 모드로 설정
	if (_motor_mode_front != Mode::Velocity && rtu->_device_address == 0x01) setMotorMode(rtu, Mode::Velocity);
	if (_motor_mode_back != Mode::Velocity && rtu->_device_address == 0x02) setMotorMode(rtu, Mode::Velocity);
	// 모터드라이버가 enable 상태가 아니면 enable 설정
	if (_motor_enabled_front != true && rtu->_device_address == 0x01) setMotorEnabled(rtu);
	if (_motor_enabled_back != true && rtu->_device_address == 0x02) setMotorEnabled(rtu);

	if (side == 0) writeSingleRegister(rtu, rtu->_device_address, 0x2088, (uint8_t*)&rpm, sizeof(rpm));
	if (side == 1) writeSingleRegister(rtu, rtu->_device_address, 0x2089, (uint8_t*)&rpm, sizeof(rpm));
}

void RS485::setMotorMode(RTU* rtu, Mode mode)
{
	if (rtu->_device_address == 0x01) _motor_mode_front = mode;
	if (rtu->_device_address == 0x02) _motor_mode_back = mode;

	writeSingleRegister(rtu, rtu->_device_address, 0x200D, (uint8_t*)&mode, sizeof(mode));
}

void RS485::setMotorEnabled(RTU *rtu)
{
	if (rtu->_device_address == 0x01) _motor_enabled_front = true;
	if (rtu->_device_address == 0x02) _motor_enabled_back = true;
	uint16_t enable = 0x0008;

	writeSingleRegister(rtu, rtu->_device_address, 0x200E, (uint8_t*)&enable, sizeof(enable));
}

void RS485::setDeviceAddress(RTU* rtu, uint16_t address)
{
	// 주소 바꾸는 패킷 전송
	writeSingleRegister(rtu, rtu->_device_address, 0x2001, (uint8_t*)&address, sizeof(address));

	// EEPROM에 저장하는 패킷 전송
	uint16_t EEPROM_value = 0x0001;
	writeSingleRegister(rtu, rtu->_device_address, 0x2010, (uint8_t*)&EEPROM_value, sizeof(EEPROM_value));
}

// 시리얼 통신 파라미터 설정은 termios.h 을 통해서 한다.
// termios 구조체를 선언한 다음, 각종 파라미터들과의 비트연산으로 설정을 키거나 끈다.
// 그리고 설정을 마친 다음 tcsetattr() 함수로 디스크립터(_rs485_fd)에 설정 내용을 저장하는 방식인 듯 하다.
ssize_t RS485::initializeRS485()
{
	_rs485_fd = open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);	// rs485 디스크립터를 연다.
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
	rs485_config.c_cc[VTIME] = 0;
	rs485_config.c_cc[VMIN] = 0;

	tcflush(_rs485_fd, TCIFLUSH);		// 디스크립터 초기화
	ret = tcsetattr(_rs485_fd, TCSANOW, &rs485_config);	// termios 구조체 설정을 디스크립터에 저장
	if (ret < 0) return ERROR;
	return OK;
}

void RS485::setRTUPacket(RTU *rtu, uint8_t device_address, uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length)
{
	rtu->_device_address = device_address;
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
	data[6] = crc & 0xFF;
	data[7] = crc >> 8;
	return crc;
}

void RS485::writeSingleRegister(RTU *rtu, uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length)
{
	setRTUPacket(rtu, device_address, 0x06, register_address, data, data_length);	// CRC를 제외한 RTU 패킷 설정
	calculateCRC((uint8_t*)rtu, sizeof(*rtu) - 2);					// CRC 계산하여 RTU 패킷 완성
	write(_rs485_fd, rtu, sizeof(*rtu));						// RTU 패킷 전송
	usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	// function code 0x06으로 write를 해도 모터드라이버에서 응답을 하여 버퍼에 쌓이므로, 이를 제거한다.
	uint8_t buf[8];
	read(_rs485_fd, &buf, sizeof(buf));
}

void RS485::readRegisters(RTU* rtu, uint16_t register_address, uint16_t register_number)
{
	setRTUPacket(rtu, rtu->_device_address, 0x03, register_address, (uint8_t*)&register_number, sizeof(register_number));
	calculateCRC((uint8_t*)rtu, sizeof(*rtu) - 2);
	write(_rs485_fd, rtu, sizeof(*rtu));
	usleep(5000);	// delay를 주지 않으면 값 전송이 제대로 안 됨.

	static uint8_t buf[9];
	ssize_t ret = read(_rs485_fd, &buf, sizeof(buf));
	if (ret <= 0) return;
	_read_buf_address = buf + 3;
}

extern "C" __EXPORT int dj_app_main(int argc, char *argv[])
{
	return RS485::main(argc, argv);

	// RS485 rs485;
	// rs485.initializeRS485();

	// rs485.readRegisters(&rs485._rtu_front, 0x2001, 0x0002);
	// printf("First data: %.2x%.2x\n", rs485._read_buf_address[0], rs485._read_buf_address[1]);
	// printf("Second data : %.2x%.2x\n", rs485._read_buf_address[2], rs485._read_buf_address[3]);
	// return 0;
}
