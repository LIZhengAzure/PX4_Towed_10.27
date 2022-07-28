/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "FixedwingAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	if (vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints。。特技操纵的最大值：
	_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
	_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));//对象子函数，而不是类的定义。
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::parameters_update() //
{
	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_k_p(_param_fw_pr_p.get());
	_pitch_ctrl.set_k_i(_param_fw_pr_i.get());
	_pitch_ctrl.set_k_ff(_param_fw_pr_ff.get());
	_pitch_ctrl.set_integrator_max(_param_fw_pr_imax.get());

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_k_p(_param_fw_rr_p.get());
	_roll_ctrl.set_k_i(_param_fw_rr_i.get());
	_roll_ctrl.set_k_ff(_param_fw_rr_ff.get());
	_roll_ctrl.set_integrator_max(_param_fw_rr_imax.get());

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_param_fw_yr_p.get());
	_yaw_ctrl.set_k_i(_param_fw_yr_i.get());
	_yaw_ctrl.set_k_ff(_param_fw_yr_ff.get());
	_yaw_ctrl.set_integrator_max(_param_fw_yr_imax.get());

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()//轮询机制update这个函数还是不好处理。
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	if (_vehicle_status.is_vtol) {
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		if (is_hovering || is_tailsitter_transition) {
			_vcontrol_mode.flag_control_attitude_enabled = false;
			_vcontrol_mode.flag_control_manual_enabled = false;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {

			if (!_vcontrol_mode.flag_control_climb_rate_enabled) {//爬升速率还没有使能的情况下，求解姿态与角速率控制。//manual、stab 情况下不进入。
				//在posi的情况下进入，给定姿态、内环速率的指令；

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs

					_att_sp.roll_body = _manual_control_setpoint.y * radians(_param_fw_man_r_max.get());

					_att_sp.pitch_body = -_manual_control_setpoint.x * radians(_param_fw_man_p_max.get())
							     + radians(_param_fw_psp_off.get());
					_att_sp.pitch_body = constrain(_att_sp.pitch_body,
								       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

					_att_sp.yaw_body = 0.0f;//姿态控制时，航向不做控制，同时航向角没有限制。
					_att_sp.thrust_body[0] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f); //固定翼的推力通道。

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();//当前时间戳

					_attitude_sp_pub.publish(_att_sp);//更新一轮角度的控制指令，相当于一次指令生成模块。

				} else if (_vcontrol_mode.flag_control_rates_enabled &&//只有角速率控制，没有姿态控制。（指令对应到角速率，类似飞行员操纵）
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual_control_setpoint.y * radians(_param_fw_acro_x_max.get());
					_rates_sp.pitch = -_manual_control_setpoint.x * radians(_param_fw_acro_y_max.get());
					_rates_sp.yaw = _manual_control_setpoint.r * radians(_param_fw_acro_z_max.get());
					_rates_sp.thrust_body[0] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);

					_rate_sp_pub.publish(_rates_sp);

				} else {
					/* manual/direct control */  //在manual，stab的情况下进入。给操纵指令。
					_actuators.control[actuator_controls_s::INDEX_ROLL] =
						_manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
					_actuators.control[actuator_controls_s::INDEX_PITCH] =
						-_manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
					_actuators.control[actuator_controls_s::INDEX_YAW] =
						_manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = math::constrain(_manual_control_setpoint.z, 0.0f, 1.0f);
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
		if (_is_tailsitter) {
			float tmp = _rates_sp.roll;
			_rates_sp.roll = -_rates_sp.yaw;
			_rates_sp.yaw = tmp;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()//固定翼着陆控制，改变着陆私有函数。
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

float FixedwingAttitudeControl::get_airspeed_and_update_scaling() //获取飞行速度并更新
{
	_airspeed_validated_sub.update();//先更新这条订阅的消息
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().calibrated_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s); //速度有效的标志：更新时间差小于1s，当前实时性？

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().calibrated_airspeed_m_s); //速度保底为0.5m/s

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the stall airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_stall.get();
		}//对于VTOL的特殊处理。这个模块总要考虑这部分的内容。
	}

	/*
	 * For scaling our actuators using anything less than the stall  //依据空速适当缩减舵面的控制输入是十分有必要的。
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	const float airspeed_constrained = constrain(constrain(airspeed, _param_fw_airspd_stall.get(),
					   _param_fw_airspd_max.get()), 0.1f, 1000.0f);//限制速度满足失速、最大之间的同时，调整速度在1000m/s以内。

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if attitude changed
	vehicle_attitude_s att; // 函数内部声明的变量。

	if (_att_sub.update(&att)) { //只要当姿态发生更新时进入循环。（更新频率？）

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {  //参数更新的标志位？ 有可能地面站的更新。
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		const float dt = math::constrain((att.timestamp - _last_run) * 1e-6f, 0.002f, 0.04f);
		_last_run = att.timestamp;

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		if (_is_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		bool wheel_control = false;

		// TODO: manual wheel_control on ground?
		if (_param_fw_w_en.get() && _att_sp.fw_control_yaw) {
			wheel_control = true;
		}

		// lock integrator if no rate control enabled, or in RW mode (but not transitioning VTOL or tailsitter), or for long intervals (> 20 ms)
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
					   !_vehicle_status.in_transition_mode && !_is_tailsitter)
				       || (dt > 0.02f);

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		//control_flaps(dt);//为什么把这个删除了？？

		float towed_att_set_roll = 0.0f;
		float towed_att_set_pitch = 0.0f;
		//float towed_att_set_yaw = 0.0f;
		/* decide if in stabilized or full manual control */
	    // 如果在要使用到flag_control_rates_enabled里面的局部变量数据，此处申请一些外部变量用于存储。
	    float pitchERROR = 0.0f; 
	    float rollERROR = 0.0f;
	    float pitchRateERROR = 0.0f;
	    float rollRateERROR = 0.0f;

		if (_vcontrol_mode.flag_control_rates_enabled) {// 在manual的时候不进入，STAB，POSCTR进入。

			const float airspeed = get_airspeed_and_update_scaling();

			/* reset integrals where needed */
			if (_att_sp.roll_reset_integral) {
				_roll_ctrl.reset_integrator();
			}

			if (_att_sp.pitch_reset_integral) {
				_pitch_ctrl.reset_integrator();
			}

			if (_att_sp.yaw_reset_integral) {
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL or tailsitter)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode && !_is_tailsitter)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}
			

			/* Prepare data for attitude controllers */
			ECL_ControlData control_input{};
			control_input.roll = euler_angles.phi();//角速率的量
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;


			if(_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL){
				control_input.roll_setpoint = _att_sp.roll_body;
				control_input.pitch_setpoint = _att_sp.pitch_body;

			}else{
				control_input.roll_setpoint  =  radians(_param_fw_rsp_off.get());
				control_input.pitch_setpoint =  radians(_param_fw_psp_off.get());

			}
			control_input.yaw_setpoint = _att_sp.yaw_body; 
			control_input.airspeed_min = _param_fw_airspd_stall.get();
			control_input.airspeed_max = _param_fw_airspd_max.get();
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;

			if (wheel_control) {
				_local_pos_sub.update(&_local_pos);

				/* Use stall airspeed to calculate ground speed scaling region.
				* Don't scale below gspd_scaling_trim
				*/
				float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
				float gspd_scaling_trim = (_param_fw_airspd_stall.get());

				control_input.groundspeed = groundspeed;

				if (groundspeed > gspd_scaling_trim) {
					control_input.groundspeed_scaler = gspd_scaling_trim / groundspeed;

				} else {
					control_input.groundspeed_scaler = 1.0f;
				}
			}

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

				} else {
					_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_yaw = _param_trim_yaw.get();

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim_roll += gradual(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_r_vmin.get(),
						     0.0f);
				trim_pitch += gradual(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_p_vmin.get(),
						      0.0f);
				trim_yaw += gradual(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_y_vmin.get(),
						    0.0f);

			} else {
				trim_roll += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						     _param_fw_dtrim_r_vmax.get());
				trim_pitch += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						      _param_fw_dtrim_p_vmax.get());
				trim_yaw += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						    _param_fw_dtrim_y_vmax.get());
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _param_fw_dtrim_r_flps.get();
			trim_pitch += _flaps_applied * _param_fw_dtrim_p_flps.get();

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(dt, control_input);//roll_rates_setpoints=(roll_setpoint-roll)/dt
					_pitch_ctrl.control_attitude(dt, control_input);

					//yaw not control
					/*
					if (wheel_control) {
						_wheel_ctrl.control_attitude(dt, control_input);
						_yaw_ctrl.reset_integrator();

					} else {
						// runs last, because is depending on output of roll and pitch attitude
						_yaw_ctrl.control_attitude(dt, control_input);
						_wheel_ctrl.reset_integrator();
					}
					*/
					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(dt, control_input);
					towed_att_set_roll = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;;
					//_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(dt, control_input);
					towed_att_set_pitch = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;;
					//_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}


					//float yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);
					/*
					if (wheel_control) {
						yaw_u = _wheel_ctrl.control_bodyrate(dt, control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);
					}
					*/
					//_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;
					//towed_att_set_yaw = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;
					/* add in manual rudder control in manual modes */
					/*
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual_control_setpoint.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					//throttle passed through if it is finite and if no engine failure was detected
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;
					*/
					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get() &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
								_battery_scale = battery_status.scale;
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}
				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

				_rate_sp_pub.publish(_rates_sp);

			} else {
				vehicle_rates_setpoint_poll();

				_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);
				_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);

				float roll_u = _roll_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

				float pitch_u = _pitch_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;

				float yaw_u = _yaw_ctrl.control_bodyrate(dt, control_input);
				_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
						_rates_sp.thrust_body[0] : 0.0f;
			}

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();

			if (wheel_control) {
				rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			} else {
				rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			}

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
			//将局部变量值输出
			pitchERROR = control_input.pitch_setpoint - control_input.pitch; // pitch_setpoint 与 pitch.
		    rollERROR = control_input.roll_setpoint - control_input.roll;    // roll_setpoint  与 roll
			pitchRateERROR = (cosf(control_input.roll) * control_input.roll_rate_setpoint +
			   					cosf(control_input.pitch) * sinf(control_input.roll) * control_input.yaw_rate_setpoint)-
								control_input.body_y_rate;
			rollRateERROR = (control_input.roll_rate_setpoint - sinf(control_input.pitch) * control_input.yaw_rate_setpoint)-
									control_input.body_y_rate;


		}

		if(_pre_nav_state != _vehicle_status.nav_state){
			_towed_y_integral = 0;
			_towed_z_integral = 0;
			_pre_nav_state = _vehicle_status.nav_state;
 		}

		switch(_vehicle_status.nav_state){

		default:
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			// test for parameters;
			//printf("**************THE MANUAL CONTROL INPUT SIGNAL*************\r\n");
			//PX4_INFO("_manual_control_setpoint.x pitch= %f", (double)_manual_control_setpoint.x);
			//PX4_INFO("_manual_control_setpoint.y roll= %f", (double)_manual_control_setpoint.y);
			//PX4_INFO("_manual_control_setpoint.z zPosition= %f", (double)_manual_control_setpoint.z);
			//PX4_INFO("_manual_control_setpoint.r yPosition= %f", (double)_manual_control_setpoint.r);

			_actuators.control[0] =  //float32[8] control.. weight power _param_fw_man_r_sc.
				_manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
			_actuators.control[1] =
				-_manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();// trim项为零。
			_actuators.control[2] = 0.f;
			_actuators.control[3] = 0.f;
			_actuators.control[4] = _manual_control_setpoint.r* _param_dlc_man_y_sc.get();
			_actuators.control[5] = (_manual_control_setpoint.z*2.0f - 1.0f)* _param_dlc_man_z_sc.get();


			//PX4_INFO("__param_dlc_man_z_sc =  %f", (double)_param_dlc_man_z_sc.get());
			//printf("**************THE ACTUATORS CONTROL SIGNALS*************\r\n");
			//PX4_INFO("_actuators.control[0] roll=  %f", (double)_actuators.control[0]);
			//PX4_INFO("_actuators.control[1] pitch= %f", (double)_actuators.control[1]);
			//PX4_INFO("_actuators.control[2] yaw= %f", (double)_actuators.control[2]);
			//PX4_INFO("_actuators.control[3] xPosition= %f", (double)_actuators.control[3]);
			//PX4_INFO("_actuators.control[3] yPosition= %f", (double)_actuators.control[4]);
			//PX4_INFO("_actuators.control[3] zPosition= %f", (double)_actuators.control[5]);
			if(_vision_position_sub.updated()){//只有当数据更新一次的时候，舵机开始适当作动。
				_vision_position_sub.copy(&_vision_position);
				_x_error = _vision_position.vision_position_x;///160.f;//mm/
				_y_error = _vision_position.vision_position_y;///160.f;
				_z_error = _vision_position.vision_position_z;//160.f;
				//测试得到的误差信息
			//printf("**************THE POSCTL ERROR original CONTROL SIGNALS*************\r\n");
				PX4_INFO("_x_error%f",(double)_x_error);
				PX4_INFO("_y_error%f",(double)_y_error);
				PX4_INFO("_z_error%f",(double)_z_error);
			}
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:


			_y_error = _manual_control_setpoint.r; // y_error [-1 1】
			_z_error = _manual_control_setpoint.z; 
			control_position_yz(dt);//PID P= 0.1 I =0.0f D=0.0f
			pitchERROR +=rollERROR +pitchRateERROR+rollRateERROR ;
			pitchERROR= 0.0f;
			
			//printf("**************THE STAB CONTROL INPUT SIGNAL*************\r\n");
			//PX4_INFO("STAB: pitch_setpoint - pitch = %f", (double)pitchERROR);
			//PX4_INFO("STAB: roll_setpoint - roll = %f", (double)rollERROR);
			//PX4_INFO("STAB: pitch_rate_setpoint - pitch_rate %f", (double)pitchRateERROR);
			//PX4_INFO("STAB: roll_rate_setpoint - roll_rate %f", (double)rollRateERROR); 
			//PX4_INFO("MANUAL: _y_error %f", (double)_y_error);
			//PX4_INFO("MANUAL: _z_error %f", (double)_z_error);
			_actuators.control[0] = -towed_att_set_roll;//将滚转指令取为负值。math::constrain(_roll_u, -1.0f, 1.0f); not _roll_u+ _roll_tirm;
			_actuators.control[1] = towed_att_set_pitch;//math::constrain(_last_output, -1.0f, 1.0f);
			_actuators.control[2] = 0.f;
			_actuators.control[3] = 0.f;
			_actuators.control[4] = _y_control;// y_control  indicate [-1 1] not sure.
			_actuators.control[5] = _z_control;

			//printf("**************THE STAB ACTUATORS CONTROL SIGNALS*************\r\n");
			//PX4_INFO("_actuators.control[0] roll=  %f", (double)_actuators.control[0]);
			//PX4_INFO("_actuators.control[1] pitch= %f", (double)_actuators.control[1]);
			//PX4_INFO("_actuators.control[2] yaw= %f", (double)_actuators.control[2]);
			//PX4_INFO("_actuators.control[3] xPosition= %f", (double)_actuators.control[3]);
			//PX4_INFO("_actuators.control[3] yPosition= %f", (double)_actuators.control[4]);
			//PX4_INFO("_actuators.control[3] zPosition= %f", (double)_actuators.control[5]); // PID ,P =0.1;
			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:


			if(_vision_position_sub.updated()){
				_vision_position_sub.copy(&_vision_position);
				_x_error = _vision_position.vision_position_x;
				_y_error = _vision_position.vision_position_y/_x_error;
				_z_error = _vision_position.vision_position_z/_x_error;
				
				

				_y_error = math::constrain(_y_error, -1.0f, 1.0f);// to longer to small...
				_z_error = math::constrain(_z_error, -1.0f, 1.0f);
				
				_auto_fuel_control.timestamp = hrt_absolute_time();
				_auto_fuel_control.control_input_error_y = _y_error;
				_auto_fuel_control.control_input_error_z = _z_error;
				_auto_fuel_topic.publish(_auto_fuel_control);



				const float dt_pos = math::constrain((_vision_position.timestamp - _last_run_pos) * 1e-6f, 0.002f, 0.05f);
				_last_run_pos = _vision_position.timestamp;
				control_position_yz(dt_pos);


				_actuators.control[0] = -towed_att_set_roll; //使用增稳控制
				_actuators.control[1] = towed_att_set_pitch; //使用增稳控制
				_actuators.control[2] = 0.f;
				_actuators.control[3] = 0.f;
				_actuators.control[4] = _y_control*_param_dlc_man_y_sc.get();//使用直接力控制的增益。
				_actuators.control[5] = _z_control*_param_dlc_man_z_sc.get();

			//printf("**************THE POSCTL ACTUATORS CONTROL SIGNALS*************\r\n");
			//PX4_INFO("_actuators.control[0] roll=  %f", (double)_actuators.control[0]);
			//PX4_INFO("_actuators.control[1] pitch= %f", (double)_actuators.control[1]);
			//PX4_INFO("_actuators.control[2] yaw= %f", (double)_actuators.control[2]);
			//PX4_INFO("_actuators.control[3] xPosition= %f", (double)_actuators.control[3]);
			//PX4_INFO("_actuators.control[3] yPosition= %f", (double)_actuators.control[4]);
			//PX4_INFO("_actuators.control[3] zPosition= %f", (double)_actuators.control[5]); // PID ,P =0.1;
			}

			break;




		}


		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		/*
		_actuators.control[actuator_controls_s::INDEX_YAW] += _param_fw_rll_to_yaw_ff.get()
				* constrain(_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual_control_setpoint.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual_control_setpoint.aux3;
		*/
		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}
	}

	perf_end(_loop_perf);
}

void 
FixedwingAttitudeControl::control_flaps (const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual_control_setpoint.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {
		flap_control = 0.5f * (_manual_control_setpoint.flaps + 1.0f) * _param_fw_flaps_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {

		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF:
			flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_lnd_scl.get();
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_to_scl.get();
			break;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? dt : -dt;

	} else {
		_flaps_applied = flap_control;
	}

	/* default flaperon to center */
	float flaperon_control = 0.0f;

	/* map flaperons by default to manual if valid */
	if (PX4_ISFINITE(_manual_control_setpoint.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		flaperon_control = 0.5f * (_manual_control_setpoint.aux2 + 1.0f) * _param_fw_flaperon_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		if (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) {
			flaperon_control = _param_fw_flaperon_scl.get();

		} else {
			flaperon_control = 0.0f;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? dt : -dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

void FixedwingAttitudeControl::control_position_yz(const float dt)
{

	// yPosition error PID control input...
	 _towed_y_integral = _towed_y_integral + _y_error *dt*_param_towed_y_i.get();// I 参数。
	if(_towed_y_integral > _param_towed_y_ilimit.get()){
		_towed_y_integral = _param_towed_y_ilimit.get();
	}
	_y_control = _param_towed_y_p.get() * _y_error +  _towed_y_integral + _param_towed_y_d.get()*_y_error/dt;

	// zPosition error PID control input...
	_towed_z_integral = _towed_z_integral + _z_error *dt*_param_towed_z_i.get();
	if(_towed_z_integral > _param_towed_z_ilimit.get()){
		_towed_z_integral = _param_towed_z_ilimit.get();
	}
	_z_control = _param_towed_z_p.get()  * _z_error +  _towed_z_integral + _param_towed_z_d.get()*_z_error/dt;

}


int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
