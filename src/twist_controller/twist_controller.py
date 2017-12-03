import pid
import lowpass
import yaw_controller
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self,
                 max_brake,
                 max_throttle,
                 max_steer_angle,
                 wheel_base,
                 steer_ratio,
                 min_speed,
                 max_lat_accel,
		 wheel_radius,
		 vehicle_mass,
                 fuel_capacity):
        self.throttle_pid = pid.PID(kp=1.5, ki=0.0, kd=0.01, mn=max_brake, mx=max_throttle)
        self.throttle_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

        self.steer_pid = pid.PID(kp=0.5, ki=0.0, kd=0.2, mn=-max_steer_angle, mx=max_steer_angle)
        self.steer_filter = lowpass.LowPassFilter(tau=0.0, ts=1.0)

        self.yaw_controller = yaw_controller.YawController(wheel_base,
                                                           steer_ratio,
                                                           min_speed,
                                                           max_lat_accel,
                                                           max_steer_angle)
        self.clk = rospy.get_time()
	self.max_acc = max_throttle
	self.max_dec = max_brake
	self.wheel_radius = wheel_radius
	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.max_brk_trq = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius * self.max_dec # -1809 N*m

    def reset(self):
        rospy.loginfo('Controller has been reset')
        self.throttle_pid.reset()
        self.steer_pid.reset()
        self.throttle_filter.ready = False
        self.steer_filter.ready = False

    def get_delta_time(self):
        t = rospy.get_time()
        dt = t - self.clk
        self.clk = t
        return dt

    def get_steer_value(self, dt, steer_cte, target_linear_vel, target_angular_vel, current_linear_vel):
        # Steering PDE
        steering = self.steer_pid.step(steer_cte, dt)

        # Yaw Controller
        yaw_control = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_linear_vel)
	#print ("yaw control cmd:", yaw_control," trg ang vel: ",target_angular_vel)

        #steer = self.steer_filter.filt(steering + 0.1 * yaw_control)
        steer = self.steer_filter.filt(0*steering + 1 * yaw_control)        
	return steer

    def get_throttle_value(self, dt, target_linear_vel, target_angular_vel, current_linear_vel, current_angular_vel):
        velocity_cte = target_linear_vel - current_linear_vel
	#print ("tar veh spd: ",target_linear_vel, "cur veh spd: ",current_linear_vel)

        throttle = self.throttle_pid.step(velocity_cte, dt)
        throttle = self.throttle_filter.filt(throttle)

        #print ("des acc: ",throttle)

        # Clamp the value between -1.0 & 1.0
        #throttle = min(1.0, max(-1.0, throttle))
        if throttle >= 0.0:
	    throttle = max(0, min(throttle, self.max_acc))	
            brake = 0.0
        else:
           #brake = -max_throttle * 1809.0
	   brake = throttle/self.max_dec * self.max_brk_trq
	   brake = min(0, max(self.max_brk_trq, brake))
           throttle = 0.0
        #rospy.loginfo("Throttle: %s" % throttle)
        #print ("throttle cmd: ", throttle," brake cmd: ", brake)	

        return throttle, brake

    def control(self,
                target_linear_vel,
                target_angular_vel,
                current_linear_vel,
                current_angular_vel,
                steer_cte):
        dt = self.get_delta_time()

        steer = self.get_steer_value(dt, steer_cte, target_linear_vel, target_angular_vel, current_linear_vel)

        throttle, brake = self.get_throttle_value(dt,
                                           target_linear_vel,
                                           target_angular_vel,
                                           current_linear_vel,
                                           current_angular_vel)
        return throttle, brake, steer



