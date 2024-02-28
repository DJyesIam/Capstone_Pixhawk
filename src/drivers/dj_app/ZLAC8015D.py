import minimalmodbus as minimodbus
import serial
import numpy as np
import time

class MotorController:

	def __init__(self, port):

		self._port = port
		self.ID = 2 # default ID : 1

		# ZLAC8015D support RS485 with Modbus RTU protocol
		self.client = minimodbus.Instrument(port, self.ID, 'rtu')
		self.client2 = minimodbus.Instrument(port, 2, 'rtu')
		self.client.serial.baudrate = 115200 # default Baudrate
		self.client.serial.parity = serial.PARITY_NONE
		self.client.serial.stopbits = 1
		self.client.serial.bytesize = 8
		self.client.serial.timeout = 1 # seconds
		
		####################
		# Register Address #
		####################

		## Common
		self.MAX_MOTOR_RPM = 0x2008
		self.CONTROL_REG = 0x200E # refer to Control CMDs
		self.OPR_MODE = 0x200D # refer to Operation Mode
		self.L_ACL_TIME = 0x2080 # Accel/Decel time (ms)
		self.R_ACL_TIME = 0x2081
		self.L_DCL_TIME = 0x2082
		self.R_DCL_TIME = 0x2083
		self.L_RATED_CUR = 0x2033
		self.L_MAX_CUR = 0x2034
		self.R_RATED_CUR = 0x2063
		self.R_MAX_CUR = 0x2064
		self.DRIVER_TEMP = 0x20B0
		self.BUS_VOLTAGE = 0x20A1

		## Velocity Control Parameters
		self.L_CMD_RPM = 0x2088 # Target RPM (RPM)
		self.R_CMD_RPM = 0x2089
		self.L_FB_RPM = 0x20AB # Actual motor RPM (0.1 rev/min)
		self.R_FB_RPM = 0x20AC

		# Torque Control Parameters
		self.L_TOQ_SLOPE = 0x2086 # Torque slope (mA/s)
		self.R_TOQ_SLOPE = 0x2087

		self.L_CMD_TOQ = 0x2090 # Target Torque (mA)
		self.R_CMD_TOQ = 0x2091

		self.L_FB_TOQ = 0x20AD # Actual Torque FB (0.1A)
		self.R_FB_TOQ = 0x20AE

		## Troubleshooting - refer to Fault code
		self.L_FAULT = 0x20A5
		self.R_FAULT = 0x20A6

		################
		# Control CMDs #
		################
		self.EMER_STOP = 0x05
		self.ALRM_CLR = 0x06
		self.DOWN_TIME = 0x07
		self.ENABLE = 0x08
		self.POS_SYNC = 0x10
		self.POS_L_START = 0x11
		self.POS_R_START = 0x12

		##################
		# Operation Mode #
		##################
		self.VEL_CONTROL = 3
		self.TOQ_CONTROL = 4

		self.ASYNC = 0
		self.SYNC = 1

		###############
		# Fault codes #
		###############
		self.NO_FAULT = 0x0000
		self.OVER_VOLT = 0x0001
		self.UNDER_VOLT = 0x0002
		self.OVER_CURR = 0x0004
		self.OVER_LOAD = 0x0008
		self.CURR_OUT_TOL = 0x0010
		self.ENCOD_OUT_TOL = 0x0020
		self.MOTOR_BAD = 0x0040
		self.REF_VOLT_ERROR = 0x0080
		self.EEPROM_ERROR = 0x0100
		self.WALL_ERROR = 0x0200
		self.HIGH_TEMP = 0x0400
		self.FAULT_LIST = [self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR, self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL, \
					self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR, self.WALL_ERROR, self.HIGH_TEMP]
		
		########################
		# Modbus function code #
		########################
		self.READ_HOLDING_REG = 0x03

		###############################
		# ZLLG80ASM250-L v3 parameter #
		###############################
		self.MAX_TORQUE = 11000 # mA
		self.RATED_TORQUE = 6000 # mA

		self.CMD_RPM = 0 # RPM

	def modbus_fail_read_handler(self, ADDR, WORD):

		read_success = False
		
		while not read_success:
			try:
				result = self.client.read_registers(ADDR, WORD, functioncode=self.READ_HOLDING_REG)
				read_success = True
			except AttributeError as e:
				print(e)
				pass

		return result
		
	def rpm_to_radPerSec(self, rpm):
		return rpm*2*np.pi/60.0

	def rpm_to_linear(self, rpm):

		W_Wheel = self.rpm_to_radPerSec(rpm)
		V = W_Wheel*self.R_Wheel

		return V

	def set_mode(self, MODE):
		if MODE == 3:
			print("Set speed rpm control")
		elif MODE == 4:
			print("Set torque mA control")
		else:
			print("set_mode ERROR: set only 3 or 4")
			return 0

		result = self.client.write_register(self.OPR_MODE, MODE)
		return result

	def get_mode(self):

		registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)
		mode = registers[0]

		return mode

	def enable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.ENABLE)

	def disable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME)

	def emergency_stop_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.EMER_STOP)

	def get_fault_code(self):

		fault_codes = self.modbus_fail_read_handler(self.L_FAULT, 2)

		L_fault_code = fault_codes[0]
		R_fault_code = fault_codes[1]

		L_fault_flag = L_fault_code in self.FAULT_LIST
		R_fault_flag = R_fault_code in self.FAULT_LIST

		return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)

	def clear_alarm(self):
		result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR)

	def set_accel_time(self, L_ms, R_ms):
		result = self.client.write_registers(self.L_ACL_TIME, [int(L_ms),int(R_ms)])

	def set_decel_time(self, L_ms, R_ms):
		result = self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)])

	def int16Dec_to_int16Hex(self,int16):

		lo_byte = (int16 & 0x00FF)
		hi_byte = (int16 & 0xFF00) >> 8
		all_bytes = (hi_byte << 8) | lo_byte

		return all_bytes

	def set_rpm(self, L_rpm, R_rpm):

		left_bytes = self.int16Dec_to_int16Hex(L_rpm)
		right_bytes = self.int16Dec_to_int16Hex(R_rpm)

		result = self.client.write_registers(self.L_CMD_RPM, [left_bytes, right_bytes])

	def get_rpm(self):
		registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
		fb_L_rpm = np.int16(registers[0])/10.0
		fb_R_rpm = np.int16(registers[1])/10.0

		return fb_L_rpm, fb_R_rpm

	def get_linear_velocities(self):

		rpmL, rpmR = self.get_rpm()

		VL = self.rpm_to_linear(rpmL)
		VR = self.rpm_to_linear(-rpmR)

		return VL, VR

	def map(self, val, in_min, in_max, out_min, out_max):
			return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def set_torque(self, L_toq, R_toq):

		left_bytes = self.int16Dec_to_int16Hex(L_toq)
		right_bytes = self.int16Dec_to_int16Hex(R_toq)

		result = self.client.write_registers(self.L_CMD_TOQ, [left_bytes, right_bytes])

	def get_torque(self):
		registers = self.modbus_fail_read_handler(self.L_FB_TOQ, 2)
		fb_L_torque = np.int16(registers[0])/10.0
		fb_R_torque = np.int16(registers[1])/10.0

		return fb_L_torque, fb_R_torque



	def set_max_rpm(self, MAX_RPM):
		max_rpm = self.int16Dec_to_int16Hex(MAX_RPM)
		result = self.client.write_register(self.MAX_MOTOR_RPM, max_rpm)
		return result
	
	def set_sync_torque(self, Toq):
		toq = self.int16Dec_to_int16Hex(Toq)
		result = self.client.write_registers(self.L_CMD_TOQ, [toq,toq])
	
	def set_rpm_w_toq(self, cmd_rpm):
		temp_cmd_rpm = cmd_rpm

		if temp_cmd_rpm == self.CMD_RPM: return 0
		else:
			self.CMD_RPM = cmd_rpm
		
		toq = self.RATED_TORQUE

		cur_L_rpm, cur_R_rpm = self.get_rpm()
		
		if cmd_rpm == 0:
			cur_L_rpm, cur_R_rpm = self.get_rpm()
			if cur_R_rpm > 0:
				self.set_sync_torque(-self.RATED_TORQUE)
				while cur_R_rpm > cmd_rpm:
					cur_L_rpm, cur_R_rpm = self.get_rpm()
					#print(f'Left RPM = {cur_L_rpm}, Right RPM = {cur_R_rpm}')
				self.set_sync_torque(0)

			elif cur_R_rpm < 0:
				self.set_sync_torque(self.RATED_TORQUE)
				while cur_R_rpm < cmd_rpm:
					cur_L_rpm, cur_R_rpm = self.get_rpm()
					#print(f'Left RPM = {cur_L_rpm}, Right RPM = {cur_R_rpm}')
				self.set_sync_torque(0)

			else:
				self.set_sync_torque(0)


		elif cmd_rpm * (cur_L_rpm + 0.01) > 0:
			toq = self.RATED_TORQUE if cmd_rpm > 0 else -self.RATED_TORQUE
			if abs(cmd_rpm) >= abs(cur_L_rpm):
				self.set_max_rpm(abs(cmd_rpm))
				self.set_sync_torque(toq)
			else:
				self.set_max_rpm(abs(cmd_rpm))
				self.set_sync_torque(-toq)
				while abs(cur_R_rpm) >= (abs(cmd_rpm)+20): # 20 is margin RPM
					cur_L_rpm, cur_R_rpm = self.get_rpm()
					#print(f'Left RPM = {cur_L_rpm}, Right RPM = {cur_R_rpm}')
				self.set_sync_torque(toq)


		elif cmd_rpm * (cur_R_rpm + 0.01) < 0:
			toq = self.RATED_TORQUE if cmd_rpm > 0 else -self.RATED_TORQUE
			self.set_max_rpm(abs(cmd_rpm))
			self.set_sync_torque(toq)

	def set_max_L_current(self, MAX_CUR):
		max_cur = self.int16Dec_to_int16Hex(MAX_CUR*10)
		result = self.client.write_register(self.L_MAX_CUR, max_cur)
		return result
	
	def set_max_R_current(self, MAX_CUR):
		max_cur = self.int16Dec_to_int16Hex(MAX_CUR*10)
		result = self.client.write_register(self.R_MAX_CUR, max_cur)
		return result
	
	def set_rated_L_current(self, rated_cur):
		cur = self.int16Dec_to_int16Hex(rated_cur*10)
		result = self.client.write_register(self.L_RATED_CUR, cur)

	def set_rated_R_current(self, rated_cur):
		cur = self.int16Dec_to_int16Hex(rated_cur*10)
		result = self.client.write_register(self.R_RATED_CUR, cur)
	
	def get_voltage(self):
		register = self.modbus_fail_read_handler(self.BUS_VOLTAGE, 1)
		vol = np.float64(register[0]/100.0)
		return vol
	
	def get_driver_temp(self):
		register = self.modbus_fail_read_handler(self.DRIVER_TEMP, 1)
		drv_temp = np.float64(register[0]/10.0)
		return drv_temp