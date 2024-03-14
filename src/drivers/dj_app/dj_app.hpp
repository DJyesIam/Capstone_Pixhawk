#pragma once

#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_encoders.h>

#define PI 3.1415

enum Mode : uint16_t
{
	INVALID = 0x0000,
	VELOCITY = 0x0003,
	TORQUE = 0x0004
};

enum RegisterAddr : uint16_t
{
// Initial Setting
	RS485_NODE_ID = 0x2001,
	OPR_MODE = 0x200D,
	CONTROL_REG = 0x200E,
	L_RATED_CUR = 0x2033,
	L_MAX_CUR = 0x2034,
	R_RATED_CUR = 0x2063,
	R_MAX_CUR = 0x2064,

// Contorl Parameters
	SAVE_EEPROM = 0x2010,
	MOTOR_MAX_RPM = 0x2008,
	L_CMD_RPM = 0x2088,
	R_CMD_RPM = 0x2089,
	L_CMD_TOQ = 0x2090,
	R_CMD_TOQ = 0x2091,

// Read Only
	DRIVER_VOL = 0x20A1,
	DRIVER_TEMP = 0x20B0,
	L_FB_RPM = 0x20AB,
	R_FB_RPM = 0x20AC,
	L_FB_TOQ = 0x20AD,
	R_FB_TOQ = 0x20AE,

// Control Word
	UNDEFINED = 0x0000,
	EMER_STOP = 0x0005,
	ALRM_CLR = 0x0006,
	ENABLE = 0x0008
};

enum PARAM : int16_t
{
	MAX_TORQUE = 13000, // mA
	RATED_TORQUE = 6000, // mA
};

struct RTU	// Modbus RTU 통신에서 주고받는 패킷을 구조체로 추상화(일단은 단일 레지스터, 단일 데이터만 주고받을 수 있는 패킷)
{
	uint8_t _node_id;
	uint8_t _function_code;
	uint8_t _register_address_high;
	uint8_t _register_address_low;
	uint8_t _data[2];
	uint8_t _crc_high;
	uint8_t _crc_low;
};

struct DriverState
{
	uint8_t node_id;
	Mode mode = Mode::INVALID;
	bool enabled = false;
	int16_t cmd_rpm = 0;
	double L_rpm, R_rpm;
	double L_vel, R_vel;
	double L_toq, R_toq;
	double voltage;
	double temperature;
};

class RS485 : public ModuleBase<RS485>, public OutputModuleInterface
{
public:
	/* ====================================== OutputModuleInterface에서 상속받은 함수들 ========================================*/
	RS485();
	~RS485() override;

	static int task_spawn(int argc, char *argv[]);	// 이 코드가 Work Queue에 올라갈 때 한 번 실행되는 함수
	static int custom_command(int argc, char *argv[]);	// 코드 기능에 영향 x
	static int print_usage(const char *reason = nullptr);	// 코드 기능에 영향 x
	int print_status() override;				// 코드 기능에 영향 x

	void Run() override;	// 실질적으로 Work Queue에서 계속 돌아가는 함수

	// 모터 출력을 업데이트하는 함수(Run 함수에서는 이 함수를 실행하지 않는데 어떻게 실행되는지 의문, 아마 밑의 _mixing_output을 업데이트할 때 실행되는 듯?)
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;
	/* =====================================================================================================================*/

	/* =============================================== 모터 관련 변수, 함수들 ==================================================*/
	DriverState _driver_broad{0x00};  // EDIT
	DriverState _driver_left{0x01};
	DriverState _driver_right{0x02};

	double _r_wheel;

	bool _drivers_initialized = false;

	void initializeDrivers();							// 모터드라이버 토크 모드, enable로 설정

	double rpmToRadPerSec(double rpm);						// rev/m 값을 rad/s로 변환
	double rpmToLinear(double rpm);							// rpm 값을 선속도 값으로 변환

	void setMode(RTU* rtu, DriverState* driver, Mode mode);				// 모터드라이버 모드 설정
	Mode getMode(RTU* rtu);								// 모터드라이버 모드 읽기
	void enableMotor(RTU* rtu, DriverState* driver);				// 모터드라이버 enable로 설정
	void disableMotor(RTU* rtu, DriverState* driver);
	void emergencyStopMotor(RTU* rtu, DriverState* driver);				// 모터드라이버 emergency stop 시키기
	void clearAlarm(RTU* rtu);							// 모터드라이버 clear fault

	void setRpm(RTU* rtu, DriverState* driver, int16_t cmd_rpm);			// target rpm을 받아 두 모터의 rpm 설정
	void getRpm(RTU* rtu, DriverState* driver);					// 두 모터의 rpm 읽기
	void getLinearVelocities(RTU* rtu, DriverState* driver);			// 두 모터의 rpm을 읽고 선속도로 변환하여 저장

	void setTorque(RTU* rtu, DriverState* driver, int16_t toq);			// target 토크를 받아 두 모터의 toq 설정
	void getTorque(RTU* rtu, DriverState* driver);					// 두 모터의 토크 읽기

	void setMaxRpm(RTU* rtu, uint16_t max_rpm);					// 모터드라이버의 최대 rpm 설정
	void setRpmWToq(RTU* rtu, DriverState* driver, int16_t cmd_rpm, int16_t margin);// 토크 모드로 두 모터의 속도 제어

	void setMaxLCurrent(RTU* rtu, uint16_t max_cur);				// 좌측 모터 최대 제한전류 설정
	void setMaxRCurrent(RTU* rtu, uint16_t max_cur);				// 우측 모터 최대 제한전류 설정
	void setRatedLCurrent(RTU* rtu, uint16_t rated_cur);				// 좌측 모터 정격전류 설정
	void setRatedRCurrent(RTU* rtu, uint16_t rated_cur);				// 우측 모터 정격전류 설정

	void getVoltage(RTU* rtu, DriverState* driver);					// 모터드라이버 인가전압 읽기
	void getDriverTemp(RTU* rtu, DriverState* driver);				// 모터드라이버 온도 읽기

	void setNodeID(RTU* rtu, uint16_t address);					// 모터드라이버 Node ID 새로 설정(주행중에는 사용하지 말 것)


	// 아마 얘가 실질적으로 모터를 돌리는 인스턴스로 추측된다.
	MixingOutput _mixing_output{"DJ", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};
	/* =====================================================================================================================*/

	/* =============================================== 통신 관련 변수, 함수들 ==================================================*/
	RTU _rtu_broad{0x00};			// EDIT
	RTU _rtu_left{0x01};			// 앞바퀴 모터드라이버 RTU 패킷(로봇 진행방향 기준 왼쪽 모터드라이버)
	RTU _rtu_right{0x02};			// 뒷바퀴 모터드라이버 RTU 패킷(로봇 진행방향 기준 오른쪽 모터드라이버)

	const char* _port_name = "/dev/ttyS3";	// 디바이스 포트 이름(pixhawk UART 포트이름이 dev/ttyS3)
	const int _baudrate = B115200;		// baudrate(모터드라이브의 default가 115200)
	// int _rs485_fd = 0;			// rs485 통신 디스크립터
	bool _rs485_initialized = false;	// rs485 통신이 초기화되었는지를 저장하는 bool 변수


	bool _motor_initialized = false;



	uint8_t* _read_buf_address;		// 모터드라이버에서 읽은 데이터 배열을 가리키는 포인터

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	int _rs485_fd{0};
	fd_set _rs485_fd_set;
	struct timeval _rs485_fd_timeout; // 최소 10ms
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	// uORB 토픽 선언(일단 roboclaw에 있는 거 넣었는데 필수인지는 모르겠음. 추후 테스트 예정)
	uORB::SubscriptionData<actuator_armed_s> _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::PublicationData<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	ssize_t initializeRS485();														 	// RS485 통신 초기화
	void setRTUPacket(RTU* rtu, uint8_t device_address, uint8_t function_code, uint16_t register_address, uint8_t* data, size_t data_length); 	// RTU 패킷 설정
	uint16_t calculateCRC(uint8_t* data, size_t data_length);											// CRC 계산
	void writeSingleRegister(RTU* rtu, uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length);			// 데이터 한 개 쓰기
	void writeSingleRegisterUsingUsleep(RTU* rtu, uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length);		// 데이터 한 개 쓰기(usleep() 함수 사용)
	void writeRegisters(uint8_t device_address, uint16_t register_address, uint8_t* data, size_t data_length);					// 데이터 두 개 쓰기
	void readRegisters(RTU* rtu, uint16_t register_address, uint16_t register_number);								// 데이터 읽기

	// 왜인지 모르겠는데 pwm_out에서 쓰길래 추가함(나중에 빼고 테스트해볼 예정)
	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	/* =====================================================================================================================*/
};

// TODO
// 브로드캐스트용 구조체 2개 만들어놓긴 했는데 이제 이걸 어떻게
// 어떤 상황에서 써먹어야 할 지 궁리 좀 해봐야 할 거 같음
// 여차하면 setRpmWToq 함수 마개조해야 할 수도
