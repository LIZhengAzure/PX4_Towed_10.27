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

//_local_pos_sub.update(&_local_pos);
//float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
//PX4_INFO("_manual_control_setpoint.r yPosition= %f", (double)_manual_control_setpoint.r);
//printf("**************THE MANUAL CONTROL INPUT SIGNAL*************\r\n");
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

					//注意，遥控器指令不直接控制姿态，采用预定值。
					_att_sp.roll_body = radians(_param_fw_rsp_off.get());
					_att_sp.pitch_body = radians(_param_fw_psp_off.get());
					_att_sp.yaw_body = radians(_param_fw_ysp_off.get());//姿态控制时，航向不做控制，同时航向角没有限制。
					_att_sp.thrust_body[0] = 0.0f; 

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();//当前时间戳

					_attitude_sp_pub.publish(_att_sp);//更新一轮角度的控制指令，相当于一次指令生成模块。

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

	vehicle_attitude_s att; // 函数内部声明的变量,只用于此次循环管体。

	if (_att_sub.update(&att)) { //只要当姿态发生更新时进入循环。将订阅信息传递到当地变量中。

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
		_last_run = att.timestamp; //内部变量，上一次循环时间

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(att.q);

		vehicle_angular_velocity_s angular_velocity{};

		_vehicle_rates_sub.copy(&angular_velocity);

		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();
		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		//_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;//TODO：航向是否控制？
		// 航向使用新的标志位进行设置。

		// lock integrator if no rate control enabled, or in RW mode (but not transitioning VTOL or tailsitter), or for long intervals (> 20 ms)
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
					   !_vehicle_status.in_transition_mode && !_is_tailsitter)
				       || (dt > 0.02f);

		float towed_att_set_roll = 0.0f;
		float towed_att_set_pitch = 0.0f;
		float towed_att_set_yaw = 0.0f;
		//float towed_att_set_yaw = 0.0f;

		// 记录欧拉角度，用于显示。
		_record_information.record_roll = euler_angles.phi();
		_record_information.record_pitch = euler_angles.theta();
		_record_information.record_yaw = euler_angles.psi();


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

			/* Prepare data for attitude controllers */
			ECL_ControlData control_input{}; // 内部变量：control_input .att/
			control_input.roll = euler_angles.phi();//角速率的量
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;

			control_input.roll_setpoint  =  _att_sp.roll_body; //radians(_param_fw_rsp_off.get());
			control_input.pitch_setpoint = _att_sp.pitch_body;// radians(_param_fw_psp_off.get());
			control_input.yaw_setpoint = _att_sp.yaw_body;//0.0f; // TODO : 应该是多少 

			control_input.airspeed_min = _param_fw_airspd_stall.get();
			control_input.airspeed_max = _param_fw_airspd_max.get();
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;

			


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

			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_yaw = _param_trim_yaw.get();
			// TODO  空速缩减量， 确定配平的极性关系。（晚上测试）
			if (_param_towed_vtrim_flag.get()){
				if (airspeed < _param_fw_airspd_trim.get()) {
				trim_pitch += gradual(airspeed, _param_fw_airspd_stall.get(), _param_fw_airspd_trim.get(), _param_fw_dtrim_p_vmin.get(),
						      0.0f);
				} 
				else {	
				trim_pitch += gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
						      _param_fw_dtrim_p_vmax.get());
				}
			}

			// fw_dtrim_r_vmax  最大车速下配平增量。

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(dt, control_input);//roll_rates_setpoints=(roll_setpoint-roll)/dt
					_pitch_ctrl.control_attitude(dt, control_input);
					_yaw_ctrl.control_attitude(dt, control_input); // 航向的姿态控制应该设定为固定值。
					
					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(dt, control_input);
					towed_att_set_roll = (PX4_ISFINITE(roll_u)) ? roll_u + (trim_roll) : (trim_roll);
					
					if (!PX4_ISFINITE(towed_att_set_roll)) {
						_roll_ctrl.reset_integrator();
					}

					float pitch_u = _pitch_ctrl.control_euler_rate(dt, control_input);
					towed_att_set_pitch = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;
					
					if (!PX4_ISFINITE(towed_att_set_pitch)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = _yaw_ctrl.control_euler_rate(dt, control_input);
					towed_att_set_yaw = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					if (!PX4_ISFINITE(towed_att_set_yaw)) {
						_yaw_ctrl.reset_integrator();
					}

				}

				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();
				_rate_sp_pub.publish(_rates_sp);

			} 

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();
			rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			_rate_ctrl_status_pub.publish(rate_ctrl_status);
			//将局部变量值输出
			//pitchERROR = control_input.pitch_setpoint - control_input.pitch; // pitch_setpoint 与 pitch.
		        //rollERROR = control_input.roll_setpoint - control_input.roll;    // roll_setpoint  与 roll
			//pitchRateERROR = (cosf(control_input.roll) * control_input.roll_rate_setpoint +
			   					//cosf(control_input.pitch) * sinf(control_input.roll) * control_input.yaw_rate_setpoint)-
								//control_input.body_y_rate;
			//rollRateERROR = (control_input.roll_rate_setpoint - sinf(control_input.pitch) * control_input.yaw_rate_setpoint)-
									//control_input.body_y_rate;
		}
		// 模态切换之后需要做的事情：TODO。
		if(_pre_nav_state != _vehicle_status.nav_state){
			_towed_y_integral = 0;
			_towed_z_integral = 0;
			_towed_ydamp_integral = 0;
			_towed_zdamp_integral = 0;
			_pre_nav_state = _vehicle_status.nav_state;

 		}
 		// 申请函数变量
 		float control_manual_0 = 0.0f;
 		float control_manual_1 = 0.0f;	
		float control_manual_4 = 0.0f;	
		float control_manual_5 = 0.0f;
		float control_stab_4 = 0.0f;
		float control_stab_5 = 0.0f;
		float control_posctl_4 = 0.0f;
		float control_posctl_5 = 0.0f;

		switch(_vehicle_status.nav_state){

		default:
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:

			control_manual_0 = -_manual_control_setpoint.y * (_param_fw_man_r_sc.get());// 设计为50%.
			control_manual_0 = (PX4_ISFINITE(control_manual_0)) ? control_manual_0 + (_param_trim_roll.get()):(_param_trim_roll.get());
			
			control_manual_1 = -_manual_control_setpoint.x * _param_fw_man_p_sc.get();
			control_manual_1 = (PX4_ISFINITE(control_manual_1)) ? control_manual_1 + _param_trim_pitch.get():_param_trim_pitch.get();
			
			control_manual_4= -_manual_control_setpoint.r*_param_dlc_man_y_sc.get();
			control_manual_4 = (PX4_ISFINITE(control_manual_4)) ? control_manual_4 + _param_dlc_man_y_trim.get():_param_dlc_man_y_trim.get();
			
			control_manual_5 = -(_manual_control_setpoint.z*2.0f - 1.0f)* _param_dlc_man_z_sc.get();
			control_manual_5 = (PX4_ISFINITE(control_manual_5)) ? control_manual_5 + _param_dlc_man_z_trim.get():_param_dlc_man_z_trim.get();
			
			//control_manual_0 = math::constrain(control_manual_0, -0.8f, 0.8f);
			//control_manual_1 = math::constrain(control_manual_1, -0.8f, 0.8f);
			//control_manual_4 = math::constrain(control_manual_4, -0.8f, 0.8f);
			//control_manual_5 = math::constrain(control_manual_5, -0.8f, 0.8f);

			_actuators.control[0] = control_manual_0; //滚转float32[8] control.. weight power _param_fw_man_r_sc.
			_actuators.control[1] = control_manual_1; //俯仰
			_actuators.control[2] = 0.f;          //偏航
			_actuators.control[3] = 0.f;              //前后飘移
			_actuators.control[4] = control_manual_4; //左右飘移
			_actuators.control[5] = control_manual_5;  //上下漂移
		
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:


			_y_error = -_manual_control_setpoint.r; // y_error [-1 1】//遥控极性更改
			_z_error = -(_manual_control_setpoint.z*2.0f - 1.0f); //遥控极性相反

			_y_error = math::constrain(_y_error, -1.0f, 1.0f);// 
			_z_error = math::constrain(_z_error, -1.0f, 1.0f);

			//control_position_yz(dt);//PID P= 0.1 I =0.0f D=0.0f 加入内、外环位置保持操纵控制。
			_distance_sensor_sub.copy(&_CFLuna_distance);
			_CFLuna_distance_Down = _CFLuna_distance.current_distance; //将北醒8米的测距传感器使用到这里测量高度。
			direct_damp_control(dt);// 包含了获取坐标 不要调换。
			direct_position_yz(dt);

			if(!_param_towed_posh_flag.get())
			{
				_y_control = 0.f;
				_z_control = 0.f;
			}

			if(!_param_towed_damp_flag.get()){
				_y_control_damp = 0.f;
				_z_control_damp = 0.f;
			}
                       

			 _y_control = math::constrain(_y_control,-0.6f,0.6f);;//操纵设置到60%
			 _z_control = math::constrain(_z_control,-0.6f,0.6f);;

                        _y_control_damp = math::constrain(_y_control_damp,-0.4f,0.4f);;//操纵设置到40%
                        _z_control_damp = math::constrain(_z_control_damp, -0.4f, 0.4f);

			
			control_stab_4 = (_y_control + _y_control_damp);
			control_stab_5 = (_z_control+ _z_control_damp);
			

			
			control_stab_4 = (PX4_ISFINITE(control_stab_4)) ? (control_stab_4 + _param_dlc_man_y_trim.get()):_param_dlc_man_y_trim.get();

			
			control_stab_5 = (PX4_ISFINITE(control_stab_5)) ? (control_stab_5 + _param_dlc_man_z_trim.get()):_param_dlc_man_z_trim.get();

			//control_stab_4 = math::constrain(control_stab_4, -0.8f, 0.8f);
			//control_stab_5 = math::constrain(control_stab_5, -0.8f, 0.8f);
			_actuators.control[0] = towed_att_set_roll;//将滚转指令取为负值。math::constrain(_roll_u, -1.0f, 1.0f); not _roll_u+ _roll_tirm;
			_actuators.control[1] = towed_att_set_pitch;//math::constrain(_last_output, -1.0f, 1.0f);
			
			//航向控制
			if (_param_towed_yctl_flag.get())
				_actuators.control[2] = towed_att_set_yaw;

			else
				_actuators.control[2] = 0.0f;
			
			_actuators.control[3] = 0.0f;
			_actuators.control[4] = control_stab_4;// add control 
			_actuators.control[5] = control_stab_5;// add control 

			break;

		case vehicle_status_s::NAVIGATION_STATE_POSCTL:


			
				_vision_position_sub.copy(&_vision_position);

				_x_error = _vision_position.vision_position_x;
				//TODO： 添加缩减因子， 对所有的数据进行归一化处理
				_y_error = _vision_position.vision_position_y/100;
				_z_error = _vision_position.vision_position_z/100;
				// 限制误差输入量为 -1 1 之间
			        _y_error = math::constrain(_y_error, -1.0f, 1.0f);// 
				_z_error = math::constrain(_z_error, -1.0f, 1.0f);

				control_position_yz(dt);

				control_posctl_4 = _y_control*_param_dlc_man_y_sc.get();
				control_posctl_4 = (PX4_ISFINITE(control_posctl_4)) ? (control_posctl_4 + _param_dlc_man_y_trim.get()):_param_dlc_man_y_trim.get();
				control_posctl_5 = _z_control*_param_dlc_man_z_sc.get();
				control_posctl_5 = (PX4_ISFINITE(control_posctl_5)) ? (control_posctl_5 + _param_dlc_man_z_trim.get()):_param_dlc_man_z_trim.get();

				//此处应该是最大的饱和位置，权限分别为50的同时，需要调整此处为0.8
				control_posctl_4 = math::constrain(control_posctl_4, -0.8f, 0.8f);
				control_posctl_5 = math::constrain(control_posctl_5, -0.8f, 0.8f);

				_actuators.control[0] = towed_att_set_roll; //使用增稳控制
				_actuators.control[1] = towed_att_set_pitch; //使用增稳控制
				_actuators.control[2] =towed_att_set_yaw;
				_actuators.control[3] = 0.f;
				_actuators.control[4] = control_posctl_4;//_param_dlc_man_y_sc.get();//使用直接力控制的增益。
				_actuators.control[5] = control_posctl_5;//_param_dlc_man_z_sc.get();	

			break;

		}
		//将控制量、记录数据进行public。
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = att.timestamp;

		//逻辑限制：只有最基础的模态存在时，进行控制发送。
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}

		_record_information.record_input_error_y = _y_error;
		_record_information.record_input_error_z = _z_error;
		_record_information.record_control_y = _y_control;
		_record_information.record_control_z = _z_control;
		_record_information.record_damp_y = _y_control_damp;
		_record_information.record_damp_z = _z_control_damp;
		_record_information.frame_y_error = _frame_y_error;
		_record_information.frame_z_error = _frame_z_error;
		_record_information.frame_x_error = _frame_x_error;
		_record_information.frame_vy_error = _frame_vy_error;
		_record_information.frame_vz_error = _frame_vz_error;
		_record_information.frame_vx_error = _frame_vx_error;
		_record_information.timestamp = hrt_absolute_time();
		_record_information_0_pub.publish(_record_information);

	}

	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_position_yz(const float dt)
{

	// yPosition error PID control input...
	// _towed_y_integral = _towed_y_integral + _y_error *dt*_param_towed_y_i.get();// I 参数。
	//if(_towed_y_integral > _param_towed_y_ilimit.get()){
	//	_towed_y_integral = _param_towed_y_ilimit.get();
	//}
	_y_control = _param_towed_y_p.get() * _y_error ;//+  _towed_y_integral + _param_towed_y_d.get()*_y_error/dt;

	// zPosition error PID control input...
	//_towed_z_integral = _towed_z_integral + _z_error *dt*_param_towed_z_i.get();
	//if(_towed_z_integral > _param_towed_z_ilimit.get()){
	//	_towed_z_integral = _param_towed_z_ilimit.get();
	//}
	_z_control = _param_towed_z_p.get()  * _z_error;// +  _towed_z_integral + _param_towed_z_d.get()*_z_error/dt;

}

void FixedwingAttitudeControl::direct_position_yz(const float dt)
{
	float frame_y_error = math::constrain(_frame_y_error, -1.0f, 1.0f);// 初始误差。
	//此处使用北醒激光测距模块的信息用于提高高度信息。因为纵向信息总是不稳定的。
	_frame_z_error = _CFLuna_distance_Down-0.14f; //问题是没有归一化处理。//设定基础高度为2米。 极性是相反的。拖曳体长度。
	float frame_z_error = math::constrain(_frame_z_error, -1.0f, 1.0f);
	float ERROR_Y = _y_error+frame_y_error; //由于激光雷达是极性相反的。因此要如此设置。
	float ERROR_Z = _z_error-frame_z_error;
	// yPosition error PID control input...
	_towed_y_integral = _towed_y_integral + ERROR_Y *dt*_param_towed_y_i.get();// I 参数。
	if(_towed_y_integral > _param_towed_y_ilimit.get()){
		_towed_y_integral = _param_towed_y_ilimit.get();
	}
	_y_control = _param_towed_y_p.get() * ERROR_Y +  _towed_y_integral;// + _param_towed_y_d.get()*ERROR_Y/dt;

	// zPosition error PID control input...
	_towed_z_integral = _towed_z_integral + ERROR_Z *dt*_param_towed_z_i.get();
	if(_towed_z_integral > _param_towed_z_ilimit.get()){
		_towed_z_integral = _param_towed_z_ilimit.get();
	}
	_z_control = _param_towed_z_p.get()  * ERROR_Z+ _towed_z_integral ;//+ _param_towed_z_d.get()*_z_error/dt;
}



void FixedwingAttitudeControl::direct_damp_control(const float dt)
{
// TODO :: 添加对应的阻尼控制模块。
	//获取反馈信号
	 _local_pos_sub.update(&_local_pos);//更新位置信息
	//float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
	//进行数据处理
	//Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
	float Towed_Head = _local_pos.heading;//不可能大于180.
	
	

	float Towed_Vx_NED = _local_pos.vx;
	float Towed_Vy_NED = _local_pos.vy;
	float Towed_Vz_NED = _local_pos.vz;

	float Towed_Ax_NED = _local_pos.ax;
	float Towed_Ay_NED = _local_pos.ay;
	float Towed_Az_NED = _local_pos.az;

	float Towed_x_NED = _local_pos.x;
	float Towed_y_NED = _local_pos.y;
	float Towed_z_NED = _local_pos.z;

	// TRANS to LOCAL POSITION
	float Towed_Vx_FRM = Towed_Vx_NED*cosf(Towed_Head) + Towed_Vy_NED*sinf(Towed_Head) ;
	float Towed_Vy_FRM = -Towed_Vx_NED*sinf(Towed_Head) + Towed_Vy_NED*cosf(Towed_Head) ;
	float Towed_Vz_FRM = Towed_Vz_NED;

	float Towed_Ay_FRM = -Towed_Ax_NED*sinf(Towed_Head) + Towed_Ay_NED*cosf(Towed_Head) ;
	float Towed_Az_FRM = Towed_Az_NED;
	
	float Towed_x_FRM = Towed_x_NED*cosf(Towed_Head) + Towed_y_NED*sinf(Towed_Head) ;
	float Towed_y_FRM = -Towed_x_NED*sinf(Towed_Head) + Towed_y_NED*cosf(Towed_Head) ;
	float Towed_z_FRM = Towed_z_NED;

	_frame_z_error = Towed_z_FRM;
	_frame_x_error = Towed_x_FRM;
	_frame_y_error = Towed_y_FRM;

	_frame_vz_error = Towed_Vz_FRM;
	_frame_vy_error = Towed_Vy_FRM;
	_frame_vx_error = Towed_Vx_FRM;

	//进行积分、比例控制。 主要以位置为主PID，速度增加阻尼。
	float frame_vy_error = math::constrain(0.5f*_frame_vy_error, -1.0f, 1.0f);
	float frame_vz_error = math::constrain(0.5f*_frame_vz_error, -1.0f, 1.0f);

	float ERROR_Y = 0.f - frame_vy_error; //目标-信号 构成负反馈。
        float ERROR_Z = 0.f - frame_vz_error; //目标-信号 构成负反馈。

	_towed_ydamp_integral = _towed_ydamp_integral + ERROR_Y*dt*_param_towed_ydamp_i.get();// I 参数。

	if(_towed_ydamp_integral > _param_towed_yd_ilimit.get()){
		_towed_ydamp_integral = _param_towed_yd_ilimit.get();
	}
	_y_control_damp = _param_towed_ydamp_p.get() * ERROR_Y +  _towed_ydamp_integral - _param_towed_ydamp_d.get()*Towed_Ay_FRM;
	// 设置限定值，不易太大
	


	_towed_zdamp_integral = _towed_zdamp_integral + ERROR_Z*dt*_param_towed_zdamp_i.get();// I 参数。
	
	if(_towed_zdamp_integral > _param_towed_zd_ilimit.get()){
		_towed_zdamp_integral = _param_towed_zd_ilimit.get();
	}
	_z_control_damp = _param_towed_zdamp_p.get() * ERROR_Z +  _towed_zdamp_integral - _param_towed_zdamp_d.get()*Towed_Az_FRM;
	//设定限定值。不易太大。
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
