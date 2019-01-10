#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define int8_t char
#define uint8_t unsigned char
#define int16_t short int
#define uint16_t unsigned short
#define int32_t int
#define uint32_t unsigned int
#define int64_t __int64
#define uint64_t unsigned __int64

#define FLOAT "float"
#define INT32_T "int32_t"

#define sync_length 7
const uint8_t header_sync[sync_length]{ 0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35 };

enum MESSAGE_TYPES {
	B = 'B',
	I = 'I',
	F = 'F',
	P = 'P',
	M = 'M',
	A = 'A',
	D = 'D',
	L = 'L',
	S = 'S',
	O = 'O',
};


struct file_header {
	uint8_t file_magic[sync_length];
	uint8_t version;
	uint64_t time_stamp;
};

struct message_header {
	uint16_t msg_size;
	uint8_t msg_type;
};

struct message_b {
	uint8_t compat_flags[8];
	uint8_t incompat_flags[8];
	uint64_t appended_offsets[3];
};

struct message_f {
	char format[2048];
};

struct message_i {
	uint8_t key_len;
	char key[1024];
	char value[1024];
};

struct message_m {
	uint8_t is_continued; 
	uint8_t key_len;
	char key[1024];
	char value[1024];
};

struct message_a {
	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[512];
};

struct message_d {
	uint16_t msg_id;
	uint8_t data[1024];
};

struct sensor_combined {
	uint64_t timestamp; 
	float gyro_rad[3]; 
	uint32_t gyro_integral_dt; 
	int32_t accelerometer_timestamp_relative; 
	float accelerometer_m_s2[3];
	uint32_t accelerometer_integral_dt; 
	uint8_t _padding0[4];
};

struct vehicle_gps_position {
	uint64_t timestamp; 
	uint64_t time_utc_usec; 
	int32_t lat; 
	int32_t lon; 
	int32_t alt; 
	int32_t alt_ellipsoid; 
	float s_variance_m_s; 
	float c_variance_rad; 
	float eph; 
	float epv; 
	float hdop; 
	float vdop; 
	int32_t noise_per_ms; 
	int32_t jamming_indicator; 
	float vel_m_s; 
	float vel_n_m_s; 
	float vel_e_m_s; 
	float vel_d_m_s; 
	float cog_rad; 
	int32_t timestamp_time_relative;
	uint8_t fix_type;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[5];
};

struct ekf2_innovations
{
	uint64_t timestamp; 
	float vel_pos_innov[6];
	float mag_innov[3];
	float heading_innov; 
	float airspeed_innov;
	float beta_innov; 
	float flow_innov[2];
	float hagl_innov; 
	float vel_pos_innov_var[6];
	float mag_innov_var[3];
	float heading_innov_var; 
	float airspeed_innov_var; 
	float beta_innov_var; 
	float flow_innov_var[2];
	float hagl_innov_var; 
	float output_tracking_error[3];
	float drag_innov[2];
	float drag_innov_var[2];
	float aux_vel_innov[2];
	uint8_t _padding0[4];
};

struct vehicle_air_data
{
	uint64_t timestamp; 
	float baro_alt_meter; 
	float baro_temp_celcius; 
	float baro_pressure_pa; 
	float rho;
};

struct vehicle_magnetometer
{
	uint64_t timestamp; 
	float magnetometer_ga[3];
	uint8_t _padding0[4];
};

struct vehicle_global_position
{
	uint64_t timestamp; 
	double lat; 
	double lon; 
	float alt; 
	float delta_alt; 
	float vel_n; 
	float vel_e; 
	float vel_d; 
	float yaw; 
	float eph; 
	float epv; 
	float terrain_alt; 
	uint8_t lat_lon_reset_counter; 
	uint8_t alt_reset_counter; 
	bool terrain_alt_valid;
	bool dead_reckoning;
};

struct input_rc {
	uint64_t timestamp; 
	uint64_t timestamp_last_signal; 
	uint32_t channel_count; 
	int32_t rssi; 
	uint16_t rc_lost_frame_count; 
	uint16_t rc_total_frame_count; 
	uint16_t rc_ppm_frame_length; 
	uint16_t values[18];
	bool rc_failsafe; 
	bool rc_lost; 
	uint8_t input_source; 
	uint8_t _padding0[3];
};

struct vehicle_status
{
	uint64_t timestamp; 
	uint32_t onboard_control_sensors_present; 
	uint32_t onboard_control_sensors_enabled; 
	uint32_t onboard_control_sensors_health; 
	uint8_t nav_state; 
	uint8_t arming_state; 
	uint8_t hil_state; 
	bool failsafe; 
	uint8_t system_type; 
	uint8_t system_id; 
	uint8_t component_id; 
	bool is_rotary_wing; 
	bool is_vtol; 
	bool vtol_fw_permanent_stab; 
	bool in_transition_mode; 
	bool in_transition_to_fw; 
	bool rc_signal_lost; 
	uint8_t rc_input_mode; 
	bool data_link_lost; 
	bool high_latency_data_link_active; 
	uint8_t data_link_lost_counter; 
	bool engine_failure; 
	bool mission_failure; 

	uint8_t _padding0;
};

struct system_power {
	uint64_t timestamp; 
	float voltage5V_v;
	float voltage3V3_v;
	uint8_t v3v3_valid; 
	uint8_t usb_connected; 
	uint8_t brick_valid; 
	uint8_t usb_valid; 
	uint8_t servo_valid; 
	uint8_t periph_5V_OC; 
	uint8_t hipower_5V_OC; 
	uint8_t _padding0;
};

struct camera_capture {
	uint64_t timestamp; 
	uint64_t timestamp_utc; 
	double lat; 
	double lon; 
	uint32_t seq; 
	float alt; 
	float ground_distance; 
	float q[4];
	int8_t result; 
	uint8_t _padding0[3];
};

struct camera_trigger {
	uint64_t timestamp; 
	uint64_t timestamp_utc; 
	uint32_t seq; 
	uint8_t _padding0[4];
};

struct safety {
	uint64_t timestamp; 
	bool safety_switch_available; 
	bool safety_off; 
	bool override_available; 
	bool override_enabled; 
	uint8_t _padding0[4];
};

struct message_l {
	uint8_t log_level;
	uint64_t timestamp;
	char message[1024];
};


void ulog_parse(const char * filename)
{
	const char * LOG_LVLS[8]{
		"EMERG",
		"ALERT",
		"CRIT",
		"ERR",
		"WARNING",
		"NOTICE",
		"INFO",
		"DEBUG"
	};

	uint8_t buffer[4096];

	FILE * debug = fopen("./OUT/log.txt", "w");
	FILE * out1 = fopen("./OUT/mems.txt", "w");
	FILE * out2 = fopen("./OUT/gps.txt", "w");
	FILE * out3 = fopen("./OUT/filter.txt", "w");
	FILE * out4 = fopen("./OUT/baro.txt", "w");
	FILE * out5 = fopen("./OUT/mag.txt", "w");
	FILE * out6 = fopen("./OUT/global.txt", "w");
	FILE * out7 = fopen("./OUT/status.txt", "w");
	FILE * out8 = fopen("./OUT/rc.txt", "w");
	FILE * out9 = fopen("./OUT/settings.txt", "w");
	FILE * out10 = fopen("./OUT/power.txt", "w");
	FILE * out11 = fopen("./OUT/output.txt", "w");

	FILE * out12 = fopen("./OUT/capture.txt", "w");
	FILE * out13 = fopen("./OUT/trigger.txt", "w");
	FILE * out14 = fopen("./OUT/safety.txt", "w");

	FILE * fp = fopen(filename, "rb");

	if (!fp) throw - 1;                         
	
	file_header f_header;
	fread(&f_header, sizeof(file_header), 1, fp);
	if (memcmp(f_header.file_magic, header_sync, sync_length) != 0)
		throw - 1;

	fprintf(debug, "0, version: %d, timestamp: %d\n", (int)f_header.version, (int)f_header.time_stamp);

	message_header m_header;
	message_a ma;
	message_b mb;
	message_f mf;
	message_i mi;
	message_d md;
	message_l ml;


	sensor_combined mems_pack;
	vehicle_gps_position gps_pack;
	ekf2_innovations filter_pack;
	vehicle_air_data baro_pack;
	vehicle_magnetometer mag_pack;
	vehicle_global_position global_pack;
	vehicle_status status_pack;
	input_rc rc_pack;
	system_power power_pack;
	camera_capture capture_pack;
	camera_trigger trigger_pack;
	safety safety_pack;

	memset(&mems_pack, 0, sizeof(sensor_combined));
	memset(&gps_pack, 0, sizeof(vehicle_gps_position));
	memset(&filter_pack, 0, sizeof(ekf2_innovations));
	memset(&baro_pack, 0, sizeof(vehicle_air_data));
	memset(&mag_pack, 0, sizeof(vehicle_magnetometer));
	memset(&global_pack, 0, sizeof(vehicle_global_position));
	memset(&status_pack, 0, sizeof(vehicle_status));
	memset(&rc_pack, 0, sizeof(input_rc));
	memset(&power_pack, 0, sizeof(system_power));
	memset(&capture_pack, 0, sizeof(camera_capture));
	memset(&trigger_pack, 0, sizeof(camera_trigger));
	memset(&safety_pack, 0, sizeof(safety));

	int counter = 0;
	int mems_id = -1, gps_id = -1, filter_id = -1, baro_id = -1, mag_id = -1, global_id = -1, status_id = -1, rc_id = -1, power_id = -1;
	int capture_id = -1, trigger_id = -1, safety_id = -1;

	while (!feof(fp))
	{
		counter++;
		fread(&m_header, sizeof(message_header), 1, fp);
		
		fprintf(debug, "%d, type: %c, size: %d\n", counter, (int)m_header.msg_type, (int)m_header.msg_size);
		switch (m_header.msg_type)
		{
		case B:
			fread(mb.compat_flags, 8, 1, fp);
			fread(mb.incompat_flags, 8, 1, fp);
			fread(mb.appended_offsets, 24, 1, fp);
			fprintf(debug, "\tcompat_flags: ");
			for (int i = 0; i < 8; i++) fprintf(debug, "%d, ", (int)mb.compat_flags[i]);
			fprintf(debug, "\n\tincompat_flags: ");
			for (int i = 0; i < 8; i++) fprintf(debug, "%d, ", (int)mb.incompat_flags[i]);
			fprintf(debug, "\n\tappended_offsets: ");
			for (int i = 0; i < 3; i++) fprintf(debug, "%I64d, ", mb.appended_offsets[i]);
			fprintf(debug, "\n");
			break;
		case I:
		case P:
			fread(&mi, 1, 1, fp);
			fread(mi.key, mi.key_len, 1, fp);
			mi.key[mi.key_len] = '\0';
			fread(mi.value, m_header.msg_size - mi.key_len - 1, 1, fp);
			mi.value[m_header.msg_size - mi.key_len - 1] = '\0';
			fprintf(debug, "\tkey: %s, value: %s\n", mi.key, mi.value);

			if (m_header.msg_type == P)
			{
				if (strncmp(FLOAT, mi.key, 5) == 0)
				{
					float tot = 0.0;
					memcpy(&tot, mi.value, sizeof(float));
					fprintf(out9, "%20s:%20f\n", mi.key + 5, tot);
				}
				else if (strncmp(INT32_T, mi.key, 7) == 0)
				{
					int32_t tot = 0;
					memcpy(&tot, mi.value, sizeof(int32_t));
					fprintf(out9, "%20s:%20d\n", mi.key + 7, tot);
				}
				else {
					throw -1;
				}
			}
			break;

		case F:
			fread(mf.format, m_header.msg_size, 1, fp);
			mf.format[m_header.msg_size] = '\0';
			fprintf(debug, "\tformat: %s\n", mf.format);
			break;
		case A:
			fread(&ma, 3, 1, fp);
			fread(ma.message_name, m_header.msg_size - 3, 1, fp);
			ma.message_name[m_header.msg_size - 3] = '\0';
			fprintf(debug, "\tmulti_id: %d, msg_id: %d, msg_nm: %s\n", ma.multi_id, ma.msg_id, ma.message_name);
			if (strcmp(ma.message_name, "sensor_combined") == 0)
				mems_id = ma.msg_id;
			else if (strcmp(ma.message_name, "vehicle_gps_position") == 0)
				gps_id = ma.msg_id;
			else if (strcmp(ma.message_name, "ekf2_innovations") == 0)
				filter_id = ma.msg_id;
			else if (strcmp(ma.message_name, "vehicle_air_data") == 0)
				baro_id = ma.msg_id;
			else if (strcmp(ma.message_name, "vehicle_magnetometer") == 0)
				mag_id = ma.msg_id;
			else if (strcmp(ma.message_name, "vehicle_global_position") == 0)
				global_id = ma.msg_id;
			else if (strcmp(ma.message_name, "vehicle_status") == 0)
				status_id = ma.msg_id;
			else if (strcmp(ma.message_name, "input_rc") == 0)
				rc_id = ma.msg_id;
			else if (strcmp(ma.message_name, "system_power") == 0)
				power_id = ma.msg_id;
			else if (strcmp(ma.message_name, "camera_capture") == 0)
				capture_id = ma.msg_id;
			else if (strcmp(ma.message_name, "camera_trigger") == 0)
				trigger_id = ma.msg_id;
			else if (strcmp(ma.message_name, "safety") == 0)
				safety_id = ma.msg_id;

			break;
		case D:
			fread(&md, 2, 1, fp);
			fread(md.data, m_header.msg_size - 2, 1, fp);
			md.data[m_header.msg_size - 2] = '\0';
			fprintf(debug, "\tmsg_id: %d, data: ", md.msg_id);
			for (int i = 0; i < m_header.msg_size - 2; i++) fprintf(debug, "%02x, ", md.data[i]);
			fprintf(debug, "\n");

			if (md.msg_id == mems_id)
			{
				memcpy(&mems_pack, md.data, m_header.msg_size - 2);
				fprintf(out1, "%I64d\t%f\t%f\t%f\t%d\t%d\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\n",
					mems_pack.timestamp,
					mems_pack.gyro_rad[0], mems_pack.gyro_rad[1], mems_pack.gyro_rad[2],
					(int)mems_pack.gyro_integral_dt,
					mems_pack.accelerometer_timestamp_relative,
					mems_pack.accelerometer_m_s2[0], mems_pack.accelerometer_m_s2[1], mems_pack.accelerometer_m_s2[2],
					(int)mems_pack.accelerometer_integral_dt,
					(int)mems_pack._padding0[0], (int)mems_pack._padding0[1], (int)mems_pack._padding0[2], (int)mems_pack._padding0[3]
				);
			}
			else if (md.msg_id == gps_id)
			{
				memcpy(&gps_pack, md.data, m_header.msg_size - 2);
				fprintf(out2, "%I64d\t%I64d\t%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					gps_pack.timestamp, gps_pack.time_utc_usec,
					gps_pack.lat, gps_pack.lon, gps_pack.alt,
					gps_pack.alt_ellipsoid, gps_pack.s_variance_m_s,
					gps_pack.c_variance_rad,
					gps_pack.eph, gps_pack.epv, gps_pack.hdop, gps_pack.vdop, gps_pack.noise_per_ms, gps_pack.jamming_indicator,
					gps_pack.vel_m_s, gps_pack.vel_n_m_s, gps_pack.vel_d_m_s, gps_pack.cog_rad, gps_pack.timestamp_time_relative,
					(int)gps_pack.fix_type, gps_pack.vel_ned_valid ? 1 : 0, (int)gps_pack.satellites_used,
					gps_pack._padding0[0], gps_pack._padding0[1], gps_pack._padding0[2], gps_pack._padding0[3], gps_pack._padding0[4]
				);
			}
			else if (md.msg_id == filter_id)
			{
				memcpy(&filter_pack, md.data, m_header.msg_size - 2);
				fprintf(out3, "%I64d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\n",
					filter_pack.timestamp, filter_pack.vel_pos_innov[0], filter_pack.vel_pos_innov[1], filter_pack.vel_pos_innov[2], filter_pack.vel_pos_innov[3], filter_pack.vel_pos_innov[4], filter_pack.vel_pos_innov[5],
					filter_pack.mag_innov[0], filter_pack.mag_innov[1], filter_pack.heading_innov, filter_pack.airspeed_innov, filter_pack.beta_innov,
					filter_pack.flow_innov[0], filter_pack.flow_innov[1], filter_pack.hagl_innov,
					filter_pack.vel_pos_innov_var[0], filter_pack.vel_pos_innov_var[1], filter_pack.vel_pos_innov_var[2], filter_pack.vel_pos_innov_var[3], filter_pack.vel_pos_innov_var[4], filter_pack.vel_pos_innov_var[5],
					filter_pack.mag_innov_var[0], filter_pack.mag_innov_var[1], filter_pack.mag_innov_var[2],
					filter_pack.heading_innov_var, filter_pack.airspeed_innov_var, filter_pack.beta_innov_var, filter_pack.flow_innov_var[0], filter_pack.flow_innov_var[1], filter_pack.hagl_innov_var,
					filter_pack.output_tracking_error[0], filter_pack.output_tracking_error[1], filter_pack.output_tracking_error[2],
					filter_pack.drag_innov[0], filter_pack.drag_innov[1], filter_pack.drag_innov_var[0], filter_pack.drag_innov_var[1],
					filter_pack.aux_vel_innov[0], filter_pack.aux_vel_innov[1],
					filter_pack._padding0[0], filter_pack._padding0[0], filter_pack._padding0[0], filter_pack._padding0[0]
				);
			}
			else if (md.msg_id == baro_id)
			{
				memcpy(&baro_pack, md.data, m_header.msg_size - 2);
				fprintf(out4, "%I64d\t%f\t%f\t%f\t%f\n", baro_pack.timestamp, baro_pack.baro_alt_meter, baro_pack.baro_temp_celcius, baro_pack.baro_pressure_pa, baro_pack.rho);
			}
			else if (md.msg_id == mag_id)
			{
				memcpy(&mag_pack, md.data, m_header.msg_size - 2);
				fprintf(out5, "%I64d\t%f\t%f\t%f\t%d\t%d\t%d\t%d\n", mag_pack.timestamp, mag_pack.magnetometer_ga[0], mag_pack.magnetometer_ga[1], mag_pack.magnetometer_ga[2],
					(int)mag_pack._padding0[0], (int)mag_pack._padding0[1], (int)mag_pack._padding0[2], (int)mag_pack._padding0[3]);
			}
			else if (md.msg_id == global_id)
			{
				memcpy(&global_pack, md.data, m_header.msg_size - 2);
				fprintf(out6, "%I64d\t%lf\t%lf\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\n",
					global_pack.timestamp, global_pack.lat, global_pack.lon, global_pack.alt, global_pack.delta_alt,
					global_pack.vel_n, global_pack.vel_e, global_pack.vel_d, global_pack.yaw, global_pack.eph,
					global_pack.epv, global_pack.terrain_alt, (int)global_pack.lat_lon_reset_counter, (int)global_pack.alt_reset_counter,
					global_pack.terrain_alt_valid ? 1 : 0, global_pack.dead_reckoning ? 1 : 0
				);
			}
			else if (md.msg_id == status_id)
			{
				memcpy(&status_pack, md.data, m_header.msg_size - 2);
				fprintf(out7, "%I64d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					status_pack.timestamp, (int)status_pack.onboard_control_sensors_present, (int)status_pack.onboard_control_sensors_enabled,
					(int)status_pack.onboard_control_sensors_health, (int)status_pack.nav_state, (int)status_pack.arming_state, (int)status_pack.hil_state,
					status_pack.failsafe ? 1 : 0, (int)status_pack.system_type, (int)status_pack.system_id, (int)status_pack.component_id, 
					status_pack.is_rotary_wing ? 1 : 0, status_pack.is_vtol ? 1 : 0, status_pack.vtol_fw_permanent_stab ? 1 : 0, status_pack.in_transition_mode ? 1 : 0,
					status_pack.in_transition_to_fw ? 1 : 0, status_pack.rc_signal_lost ? 1 : 0, (int)status_pack.rc_input_mode, status_pack.data_link_lost ? 1 : 0,
					status_pack.high_latency_data_link_active ? 1 : 0, (int)status_pack.data_link_lost_counter, status_pack.engine_failure ? 1 : 0,
					status_pack.mission_failure ? 1 : 0, (int)status_pack._padding0
				);
			}
			else if (md.msg_id == rc_id)
			{
				memcpy(&rc_pack, md.data, m_header.msg_size - 2);
				fprintf(out8, "%I64d\t%I64d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					rc_pack.timestamp,
					rc_pack.timestamp_last_signal,
					(int)rc_pack.channel_count,
					rc_pack.rssi,
					(int)rc_pack.rc_lost_frame_count,
					(int)rc_pack.rc_total_frame_count,
					(int)rc_pack.rc_ppm_frame_length,
					(int)rc_pack.values[0], (int)rc_pack.values[1], (int)rc_pack.values[2], (int)rc_pack.values[3], 
					(int)rc_pack.values[4], (int)rc_pack.values[5], (int)rc_pack.values[6], (int)rc_pack.values[7],
					rc_pack.rc_failsafe ? 1 : 0, rc_pack.rc_lost ? 1 : 0, (int)rc_pack.input_source, (int)rc_pack._padding0[0], (int)rc_pack._padding0[1], (int)rc_pack._padding0[2]
				);
			}
			else if (md.msg_id == power_id)
			{
				memcpy(&power_pack, md.data, m_header.msg_size - 2);
				fprintf(out10, "%I64d\t%f\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					power_pack.timestamp, power_pack.voltage5V_v, power_pack.voltage3V3_v,
					power_pack.v3v3_valid, power_pack.usb_connected, power_pack.brick_valid,
					power_pack.usb_valid, power_pack.servo_valid, power_pack.periph_5V_OC,
					power_pack.hipower_5V_OC, power_pack._padding0
				);
			}
			else if (md.msg_id == capture_id)
			{
				memcpy(&capture_pack, md.data, m_header.msg_size - 2);
				fprintf(out12, "%I64d\t%I64d\t%12.9lf\t%12.9lf\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%d\n",
					capture_pack.timestamp, capture_pack.timestamp_utc, capture_pack.lat, capture_pack.lon,
					(int)capture_pack.seq, capture_pack.alt, capture_pack.ground_distance,
					capture_pack.q[0], capture_pack.q[1], capture_pack.q[2], capture_pack.q[3],
					capture_pack.result
				);
			}
			else if (md.msg_id == trigger_id)
			{
				memcpy(&trigger_pack, md.data, m_header.msg_size - 2);
				fprintf(out13, "%I64d\t%I64d\t%d\n",
					trigger_pack.timestamp, trigger_pack.timestamp_utc, trigger_pack.seq
				);
			}
			else if (md.msg_id == safety_id)
			{
				memcpy(&safety_pack, md.data, m_header.msg_size - 2);
				fprintf(out14, "%I64d\t%d\t%d\t%d\t%d\n",
					safety_pack.timestamp, safety_pack.safety_switch_available ? 1 : 0,
					safety_pack.safety_off ? 1 : 0, safety_pack.override_available ? 1 : 0,
					safety_pack.override_enabled ? 1 : 0
				);
			}
			break;
		case L:
			fread(&ml, 1, m_header.msg_size, fp);
			ml.message[m_header.msg_size - 9] = '\0';
			ml.log_level -= '0';
			fprintf(out11, "%9I64d\t%7s\t%s\n", ml.timestamp, LOG_LVLS[ml.log_level], ml.message);
			break;
		default:
			fread(buffer, m_header.msg_size, 1, fp);
			break;
		}
	}
	fclose(fp);
	fclose(debug);
	fclose(out1);
	fclose(out2);
	fclose(out3);
	fclose(out4);
	fclose(out5);
	fclose(out6);
	fclose(out7);
	fclose(out8);
	fclose(out9);
	fclose(out10);
	fclose(out11);
	fclose(out12);
	fclose(out13);
	fclose(out14);
}