from controller import Robot
import math

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        # Setup for wheels
        self.timestep = int(self.getBasicTimeStep())
        self.wheels = [self.getDevice("wheel" + str(i)) for i in range(1, 5)]
        self.detected_box_color = None
        self.is_picking_up = False
        self.pickup_state = 'align'
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

        self.max_velocity = 14.81
        self.MOVEMENT_VELOCITY = 12
        self.movement_velocity = 6.28

        # Setup for sensors
        self.sensors_index = [1, 2, 3, 4, 5, 6, 7, 8,9,10,11,12]
        self.sensors = []
        self.sensors_coefficient = [5000,4500,4000, 3000, 2000, 1000, -1000, -2000, -3000, -4000,-4500,-5000]
        for index in self.sensors_index:
            sensor = self.getDevice("ir" + str(index))
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
        self.armMotors = []
        self.armMotors.append(self.getDevice("arm1"))
        self.armMotors.append(self.getDevice("arm2"))
        self.armMotors.append(self.getDevice("arm3"))
        self.armMotors.append(self.getDevice("arm4"))
        self.armMotors.append(self.getDevice("arm5"))
        self.armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
        self.armMotors[1].setVelocity(1.5)
        self.armMotors[2].setVelocity(1.5)
        self.armMotors[3].setVelocity(0.5)
        self.armMotors[4].setVelocity(1.5)


        self.armPositionSensors = []
        self.armPositionSensors.append(self.getDevice("arm1sensor"))
        self.armPositionSensors.append(self.getDevice("arm2sensor"))
        self.armPositionSensors.append(self.getDevice("arm3sensor"))
        self.armPositionSensors.append(self.getDevice("arm4sensor"))
        self.armPositionSensors.append(self.getDevice("arm5sensor"))
        for sensor in self.armPositionSensors:
            sensor.enable(self.timestep)

        #! Initialize gripper motors.
        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5) # 0.03
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()
        
        # PID coefficients
        self.Kp = 0.0015
        self.Kd = 0.00015
        self.last_error = 0

        # Initialize wheel motors
        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)

        self.COUNTER_PICKK_UP=0
        self.COUNTER_REV_PICKK_UP=0
        self.WALL_THRESHOLD = 900
        self.MOVEMENT_VELOCITY = 12
        self.turn_speed = math.pi / 2
        self.base_speed = 10.28
        self.turn_duration = 1500  # Placeholder: the duration of a turn (to be calibrated)
        self.turn_around_duration = 3000  # Placeholder: the duration of a 180-degree turn (to be calibrated)
        self.step(self.timestep)

    
    def set_wheel_speeds(self, fl, fr, bl, br):
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(fr)
        self.back_left_motor.setVelocity(bl)
        self.back_right_motor.setVelocity(br)
    
    
    def read_sensors_value(self):
        """Read values from line-following sensors and compute weighted sum."""
        value = 0
        active_sensors = 0  # Count active sensors to normalize
        for index, sensor in enumerate(self.sensors):
            sensor_value = sensor.getValue()
            if sensor_value > 200:  # Adjusted threshold
                value += self.sensors_coefficient[index]
                active_sensors += 1
        if active_sensors > 0:
            value /= active_sensors  # Normalize the sensor contribution
        return value


    def clamp_speed(self, speed):
        return max(min(speed, self.max_velocity), -self.max_velocity)
    

    def forward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(7.0) # maxVelocity = 14.81
        self.step(time * self.timestep)

    def backward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(-1.0) # maxVelocity = 14.81
        self.step(time * self.timestep)    

    def move_backward(self, velocity):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_forward(self, velocity):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)        
        

    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def halt(self):
        for wheel in self.wheels:
            wheel.setVelocity(0.0)


    def steering(self, left_speed, right_speed):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)

        self.front_right_wheel.setVelocity(right_speed)
        self.back_right_wheel.setVelocity(right_speed)
        self.front_left_wheel.setVelocity(left_speed)
        self.back_left_wheel.setVelocity(left_speed)

    def line_follow(self):
        goal = 0
        reading = self.read_sensors_value()
        error = goal - reading
        P = self.Kp * error

        error_rate = error - self.last_error
        self.last_error = error
        D = self.Kd * error_rate

        steering_correction = P + D
        right_speed = self.movement_velocity / 2 - steering_correction
        left_speed= self.movement_velocity / 2 + steering_correction
        self.steering(left_speed, right_speed)
        
        detect_fork = self.detect_fork()
        if detect_fork:
            print("switch to move linefollow" )
            return True
        return False

        


    def detect_fork(self):
        """Detect if the robot has reached a T-shaped fork in the path."""
        sensor_values = [sensor.getValue() for sensor in self.sensors]

        # Define threshold for black line detection
        black_threshold = 480

        # Count the number of sensors detecting black
        black_count = sum(value > black_threshold for value in sensor_values)

        # Define a threshold for considering it as a fork
        fork_threshold = 8 # At least 8 sensors must detect black

        if black_count >= fork_threshold:
            print(f"T-shaped fork detected! Sensor values: {sensor_values}")
            return True
        else:
            return False

    def is_line_left(self):
        """Check if the black line is detected on the left side."""
        sensor_values = [sensor.getValue() for sensor in self.sensors]
    
        # Define threshold for black line detection
        black_threshold = 480
        
        # Adjust the range to include the correct "left" sensors
        # Assuming the first 3 sensors are on the left
        left_detected = any(value > black_threshold for value in sensor_values[:3])
        
        if left_detected:
            print(f"Line detected on the left! Sensor values: {sensor_values[:3]}")
            return True
        return False
    
    
    
    def is_line_right(self):
        """Check if the black line is detected on the right side."""
        sensor_values = [sensor.getValue() for sensor in self.sensors]

        # Define threshold for black line detection
        black_threshold = 480

        # Check if the right sensors (assume second half) detect black
        right_detected = any(value > black_threshold for value in sensor_values[-3:])
        if right_detected:
            print(f"Line detected on the right! Sensor values: {sensor_values[-3:]}")
            return True
        return False
    
    
    
    
    def handle_fork_left(self):
        """Handle the fork by performing a smooth right turn."""
        print("left turn")
        turn_duration = 1500  # in ms
        num_steps = turn_duration // self.timestep

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            self.move_left(speed)
            self.step(self.timestep )

        # Reset PID controller
        self.last_error = 0
    
    
    
    
    
    
    def handle_fork_right(self):
        """Handle the fork by performing a smooth right turn."""
        print(" right turn.")



        # Total duration for the turn (adjust based on testing)
        turn_duration = 1500  # in ms
        num_steps = turn_duration // self.timestep

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            self.turn_cw(speed)
            self.step(self.timestep )

        print(" turn complete. Re-enabling sensors.")
        


        # self.halt()
        # self.step(200)

        #     # Reset wheel speeds to standard line-following speed
        # self.front_right_wheel.setVelocity(self.movement_velocity / 2)
        # self.back_right_wheel.setVelocity(self.movement_velocity / 2)
        # self.front_left_wheel.setVelocity(self.movement_velocity / 2)
        # self.back_left_wheel.setVelocity(self.movement_velocity / 2)
        print ("loo.olololololololololololololololol")
        # Reset PID controller
        self.last_error = 0

        print("Sensors re-enabled. Resuming line following.")
    
    
    
    
    
    
    
    def handle_fork(self):
        """Handle the fork by performing a smooth right turn."""
        print("Handling fork... Disabling sensors and initiating smooth right turn.")

        # Disable sensors
        for sensor in self.sensors:
            sensor.disable()

        # Total duration for the turn (adjust based on testing)
        turn_duration = 1500  # in ms
        num_steps = turn_duration // self.timestep

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            # self.move_left(speed)
            #self.turn_cw(speed)
            self.move_right(speed)
            self.step(self.timestep)

        print(" turn complete. Re-enabling sensors.")

        # Re-enable sensors
        for sensor in self.sensors:
            sensor.enable(self.timestep)

        print("Sensors re-enabled. Resuming line following.")


    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)


    def turn_90(self):
            """Handle the fork by performing a smooth right turn."""
            print("turning 90 degree")

            turn_duration = 5800# in ms
            num_steps = turn_duration // self.timestep
            

            for step in range(num_steps):
                factor = math.sin(math.pi * step / num_steps)

                speed = 10.97 * factor
                self.front_left_wheel.setVelocity(self.max_velocity)
                self.back_left_wheel.setVelocity(self.max_velocity)
                self.front_right_wheel.setVelocity(-self.max_velocity)
                self.back_right_wheel.setVelocity(-self.max_velocity)
                self.step(self.timestep )
                self.halt()

            print(" turn 90 complete")


    def turn_minus_90(self):
        """Handle the fork by performing a smooth right turn."""
        print("turning -90 degree")

        turn_duration = 5800# in ms
        num_steps = turn_duration // self.timestep
        

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = 13.97 * factor
            self.front_left_wheel.setVelocity(-speed)
            self.back_left_wheel.setVelocity(-speed)
            self.front_right_wheel.setVelocity(speed)
            self.back_right_wheel.setVelocity(speed)
            self.step(self.timestep )
            self.halt()

        print(" turn complete")


    def handle_180_degree_turn_start(self):
        """Perform a 180-degree turn by calling handle_fork_right twice."""
        print("Starting 180-degree turn...")

        # First 90-degree turn
        self.turn_90()

        #self.step(7000)
        #self.move_backward(10.0)
        self.backward(12)
        

        # Second 90-degree turn
        self.turn_90()

        print("180-degree turn complete.")

    def handle_180_degree_turn(self):
        """Perform a 180-degree turn by calling handle_fork_right twice."""
        print("Starting 180-degree turn...")

        # First 90-degree turn
        self.turn_90()
        self.backward(26)
        # self.step(7000)

        # Second 90-degree turn
        self.turn_90()

        print("180-degree turn complete.")    


    def handle_minus_180_degree_turn(self):
        """Perform a -180-degree turn by calling handle_fork_right twice."""
        print("Starting -180-degree turn...")

        # First 90-degree turn
        self.turn_minus_90()
        self.minus_backward(14)
        # self.step(7000)

        # Second 90-degree turn
        self.turn_minus_90()

        print("-180-degree turn complete.")   

        
    def handle_180_degree_turn_green(self):
        """Perform a 180-degree turn by calling handle_fork_right twice."""
        print("Starting 180-degree turn...")

        # First 90-degree turn
        self.turn_90()
        self.backward(14)
        # self.step(7000)

        # Second 90-degree turn
        self.turn_90()

        print("180-degree turn complete.")   
         

    def handle_minus_180_degree_turn_yellow(self):
        """Perform a -180-degree turn by calling handle_fork_right twice."""
        print("Starting -180-degree turn...")

        # First 90-degree turn
        self.turn_minus_90()
        self.minus_backward(25)
        # self.step(7000)

        # Second 90-degree turn
        self.turn_minus_90()

        print("-180-degree turn complete.")        





    def detect_box(self):
        """Detect the closest box in the camera's recognition objects."""
        objects = self.camera.getRecognitionObjects()
        print(f"Number of objects detected: {len(objects)}")

        for obj in objects:
            model_label = obj.getModel()
            print(f"Detected object model: {model_label}") 

            colors = obj.getColors()
            red, green, blue = colors[0], colors[1], colors[2]
            current_color = (red, green, blue)
            self.detected_box_color = current_color
            # print(f"Detected box: {self.detected_box_color}") 
            # print(f"Box detected with color: R={red:.2f}, G={green:.2f}, B={blue:.2f}")

            if model_label == 'box':
                x, y = obj.getPositionOnImage()
                image_width = self.camera.getWidth()
                position = (x - (image_width / 2)) / (image_width / 2)  # Normalized [-1, 1]
                distance = abs(obj.getPosition()[0])

                if green == 1 and red == 0 and blue == 0:
                    print(f"Green box detected at distance {distance}")
                    if distance <= 0.4:
                        return 'change_direction', position, distance

                # elif red == 1 and green == 1 and blue == 0:  
                #     print(f"Yellow box detected at distance {distance}")
                #     if distance <= 0.4:
                #         return 'change_direction', position, distance

                # elif red == 1 and green == 0 and blue == 0:
                #     print(f"Red box detected at distance {distance}")
                #     if distance <= 0.4:
                #         return 'change_direction', position, distance

                # elif red == 0 and green == 0 and blue == 1:
                #     print(f"Blue box detected at distance {distance}")
                #     if distance <= 0.4:
                #         return 'change_direction', position, distance

                else:
                    if distance <= 0.35:
                        return 'go_and_catch', position, distance

        return 'no_box', 0, float("inf")


    def disable_line_sensors(self):
        for sensor in self.sensors:
            sensor.disable()

    def enable_line_sensors(self):
        for sensor in self.sensors:
            sensor.enable(self.timestep)


    

    def approach_box(self, position, distance):
        print(f"the Distance is {distance}")
        pickup_distance = 0.13
        if distance <= pickup_distance:
            self.COUNTER_PICKK_UP= self.COUNTER_PICKK_UP+1
            print(f"COUNTER_PICKK_UP is : {self.COUNTER_PICKK_UP}")
            self.perform_pick_and_place()
        else:
            turn_direction = 1 if position > 0 else -1
    
            turn_speed = self.max_velocity * turn_direction * 0.5
            move_speed = self.max_velocity * 0.5
    
            self.steering(turn_speed, -turn_speed)
            self.forward(1)
    
            self.detect_box()
            # self.line_follow()



    def approach_boxwall(self, position, distance):
        print(f"the Distance is {distance}")
        pickup_distance = 0.2
        if distance <= pickup_distance:
            self.COUNTER_PICKK_UP= self.COUNTER_PICKK_UP+1
            print(f"COUNTER_PICKK_UP is : {self.COUNTER_PICKK_UP}")
            self.perform_pick_and_place()
            
        else:
            turn_direction = 1 if position > 0 else -1
    
            turn_speed = self.max_velocity * turn_direction * 0.5
            move_speed = self.max_velocity * 0.5
    
            self.steering(-turn_speed, turn_speed)
            self.forward(1)
    
            self.detect_box()
            # self.line_follow()

    # hereeeeeeeeeeeeeeeeeeeee
    def perform_pick_and_place(self):
        self.halt()
        # self.step(100 * self.timestep)  # Wait a moment
        
        # self.pick_up()
        self.pick_up_boxwall()
        self.step(100 * self.timestep)  # Wait a moment
        # self.turn_around(0)
        self.drop()
        # self.forward(400)
        # self.turn_around(70)
        self.step(200 * self.timestep)  # Wait a moment
        self.open_grippers()
        self.step(50 * self.timestep)  # Wait a moment
        self.hand_up()
        print("End Pick Up")

    
    def pick_up(self):
        
        self.pickup_state = 'open_gripper'
        print("open_gripper")
        self.step(50 * self.timestep)  
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        
        self.pickup_state = 'lower_arm'
        print("lower arm")
        self.step(100 * self.timestep) 
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[2].setPosition(-1.08)
        self.armMotors[3].setPosition(-1.14)
        
        self.pickup_state = 'close_gripper'
        print("close gripper")
        self.step(100 * self.timestep)  
        self.finger1.setPosition(0.01)  
        print(5555555555555)
        self.finger2.setPosition(0.01)
        print("000000000000000")
        self.step(50 * self.timestep)  
        
        self.pickup_state = 'lift_arm'
        print("lift arm")
        self.step(50 * self.timestep)  
        self.armMotors[1].setPosition(0)
        print("finish")
        self.pickup_state = 'complete'
        print("complete")
        self.step(50 * self.timestep)  # Wait a moment
        self.pickup_state = 'idle'
        print("idle")


        
    def pick_up_boxwall(self):
        
        

        self.pickup_state = 'open_gripper'
        print("open_gripper")
        self.step(200 * self.timestep)  
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        
        self.pickup_state = 'lower_arm'
        print("lower arm")
        self.step(100 * self.timestep) 
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[2].setPosition(-0.7)
        self.armMotors[3].setPosition(-1.25)
        
        self.pickup_state = 'close_gripper'
        print("close gripper")
        self.step(100 * self.timestep)  
        self.finger1.setPosition(0.01)  
        print(44444444444444444)
        self.finger2.setPosition(0.01)
        print("33333333333333")
        self.step(50 * self.timestep)  
        
        self.pickup_state = 'lift_arm'
        print("lift arm")
        self.step(50 * self.timestep)  
        self.armMotors[1].setPosition(0)
        print("finish")
        self.pickup_state = 'complete'
        print("complete")
        self.step(50 * self.timestep)  # Wait a moment
        self.pickup_state = 'idle'
        print("idle")





    def open_grippers(self):
        max_position = min(self.fingerMaxPosition, 0.025)  # Assuming 0.025 is the max limit
        self.finger1.setPosition(max_position)
        self.finger2.setPosition(max_position)
        self.step(70 * self.timestep)


    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
        self.finger1.setPosition(self.fingerMinPosition)
        self.finger2.setPosition(self.fingerMinPosition)

    def forward(self,time):
            for wheel in self.wheels:
                wheel.setVelocity(7.0) # maxVelocity = 14.81
            self.step(time * self.timestep)


    def drop(self):
        print(f"COUNTER_PICKK_UP{self.COUNTER_PICKK_UP }")
        if self.COUNTER_PICKK_UP == 1 :
            self.armMotors[0].setPosition(0)
            self.armMotors[1].setPosition(0.6)
            self.armMotors[2].setPosition(0.9)
            self.armMotors[3].setPosition(1.5)
            self.armMotors[4].setPosition(0)
            
        elif self.COUNTER_PICKK_UP == 2 :
            self.armMotors[0].setPosition(2.9)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-1)
            self.armMotors[2].setPosition(-1.7)
            self.armMotors[4].setPosition(2.9)

        elif self.COUNTER_PICKK_UP == 3 :
            self.armMotors[0].setPosition(2.9)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-1)
            self.armMotors[2].setPosition(-1.7)
            self.armMotors[4].setPosition(2.9)

    
    def detect_blue_floor(self ):
        objects = self.camera.getRecognitionObjects()
        distance_x=10000
        distance_y=10000
        distance_z=10000

        for obj in objects:
            model_label = obj.getModel() 
            # if model_label != 'box' and model_label.lower() != 'rectangular panel':
            print(f"Detected object model: {model_label}")
            if model_label == 'blueee':
                
                print("1")
                distance_x = abs(obj.getPosition()[0] )         
                distance_y = abs(obj.getPosition()[1] ) 
                distance_z = abs(obj.getPosition()[2] ) 

                print(f"distance_x is :  {distance_x}\n") 
                print(f"distance_y is :  {distance_y}\n") 
                print(f"distance_z is :  {distance_z}\n") 

                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
            # Define thresholds for red color detection
                if red == 0 and green == 0 and blue == 1:
                    # if distance_x <= 0.08 and distance_y <= 0.025 and distance_z  <= 0.0037 :
                        print("2")
                        return distance_x , distance_y ,distance_z
        return distance_x , distance_y ,distance_z

    def rev_take_box_from_back(self):
        self.COUNTER_REV_PICKK_UP=self.COUNTER_REV_PICKK_UP+1
        print("halt")
        self.halt()
        self.step(100 * self.timestep)
        print("pick_up")
        self.rev_pick_up()
        self.step(100 * self.timestep)
        print("close_grippers")
        self.rev_close_grippers()
        self.step(100 * self.timestep)
        print("hand_up")
        self.rev_hand_up()
        # forward(20)
        # halt()
        self.step(100 * self.timestep)
        print("drop")
        self.rev_drop()
        # hand_up()
        self.step(100 * self.timestep)
        print("fold_arms")
        self.rev_fold_arms()
        
        
        
    def rev_pick_up(self):
        
        if self.COUNTER_REV_PICKK_UP == 1:
            self.armMotors[0].setPosition(0)
            self.armMotors[1].setPosition(0.6)
            self.armMotors[2].setPosition(1.0)
            self.armMotors[3].setPosition(1.25)
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
        elif self.COUNTER_REV_PICKK_UP == 2:
            self.armMotors[0].setPosition(2.90)
            self.armMotors[1].setPosition(-0.25)
            self.armMotors[2].setPosition(-1.35)
            self.armMotors[3].setPosition(-1.30)
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
    
    def rev_close_grippers(self):
        self.finger1.setPosition(0.013)     # Close gripper.
        self.finger2.setPosition(0.013)
    
    def rev_hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
    
    
    def rev_fold_arms(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
    
    def rev_drop(self):
        # Move arm down
        self.armMotors[3].setPosition(-1.14)
        self.armMotors[2].setPosition(-1)
        self.armMotors[1].setPosition(-1.13)
        self.step(100 * self.timestep)


        # Open gripper.
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)

    def detect_and_process_colors(self):
        """Detect the colors of the boxes in front of the robot and process them in order."""
        # Get the recognized objects from the camera
        objects = self.camera.getRecognitionObjects()
        print(f"Number of objects detected: {len(objects)}")

        # Filter out only the boxes and extract their colors and distances
        matrix = []
        for obj in objects:
            if obj.getModel() == 'matrix':  # Check if the object is a box
                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
                distance = abs(obj.getPosition()[0])  # Distance along the x-axis
                matrix.append({
                    'color': (red, green, blue),
                    'distance': distance
                })

        # Sort the boxes by distance (closest first)
        matrix.sort(key=lambda x: x['distance'])

        # Extract the colors in order
        colors_in_order = [item['color'] for item in matrix]
        print(f"Colors in order: {colors_in_order}")

        # Set the detected color to the first color in the sorted list
        if colors_in_order:
            self.detected_box_color = colors_in_order[0]
            print(f"Detected color: {self.detected_box_color}")
            return self.detected_box_color
        else:
            print("No colors detected.")
            return None
        

    def handle_forks_to_blue_cube(self):
        fork_counter = 0
        num_of_box = 0
        flag = True
        flag2 = True
        stage_5_completed = False
        stop_counter = 0  # عداد للتحكم في الإيقاف
        stop_limit = 305
        while self.step(self.timestep) != -1:
        # Step 1: Line follow until a fork is detected
            print("Line following until a fork is detected...")
            if(flag):
                self.line_follow()

            if fork_counter == 0:
                if self.detect_fork():
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.detect_fork():
                        self.handle_fork_left()
                        self.step(self.timestep)
                    fork_counter += 1
                
            
                
            elif fork_counter == 1  :
                if self.detect_fork():
                    print("Second fork detected. Handling with handle_fork_right().")
                    while self.detect_fork():
                        self.handle_fork_right()
                        self.step(self.timestep)
                        
                    print("Reached the blue cube. Performing the blue job.")
                    fork_counter += 1
                    print("forkkkkkkkkkkkkkkkkkkk",fork_counter)
                    
            elif fork_counter == 2 :
                
                print(666666666666666666666666666666666666666666666)
                print("Detecting blue floor to place cubes...")
                distance_x, distance_y, distance_z = self.detect_blue_floor()
                # print("0000000000000000"distance_x)
    # Check if the robot is close to the blue floor
                if distance_x <= 0.2:  
                    print("Blue floor detected. Placing the cube...")
                    self.halt()  
                    
                    self.rev_take_box_from_back()  
                    print("Cube placed on the blue floor.")
                    
                    print("Moving to the next stage...")
                    fork_counter += 1
                    print("Fork counter incremented. New value:", fork_counter)
                
            
            elif fork_counter == 3 :
                print("the fork count is===",fork_counter)
                
                print("Starting line following after 180-degree turn...")
                flag = False
                self.handle_180_degree_turn()
                # self.halt()
                # self.handle_180_degree_turn()
                flag = True
                self.move_forward(14.81)
                fork_counter += 1
                transition_counter = 0  # إعادة تعيين العداد
                
                print("end 180 rotate")
                
                
                
                
            elif fork_counter == 4:
            # زيادة عداد الانتقال خلال هذه المرحلة
                transition_counter += 0.01
                print(f"Transition counter: {transition_counter}")

                # تحقق من انتهاء الانتقال
                if transition_counter >= 4.50:  # يمكنك تعديل القيمة حسب الحاجة
                    print("Transition to state 5 allowed.")
                    fork_counter += 1
                    continue  # الانتقال إلى الحالة الخامسة

                if self.is_line_left():
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.is_line_left():
                        self.handle_fork_left()
                        self.step(self.timestep)

                    print("forrrrrrrrrrrrrrrrrrrrrrrrrrrkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
                
                    
            elif fork_counter ==5:
                if self.is_line_right():
                    # flag2 = False
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.is_line_right():
                        self.handle_fork_right()
                        self.step(self.timestep)
                    stage_5_completed = True  # هنا تحدد أنك انتهيت من المرحلة 5
                if stage_5_completed:
                    print("Stage 5 completed. Moving to stage 6...")
                    fork_counter += 1  # الانتقال إلى المرحلة 6
                    continue  # الانتقال إلى المرحلة التالية
                
                
            
            elif fork_counter ==6:
                print("reverse the box from back to the wall")
                stop_counter += 1  # زيادة العداد في كل خطوة
                print(f"Stop counter: {stop_counter}")

                if stop_counter >= stop_limit:
                    print("Stop limit reached. Halting the robot.")
                    self.halt()
                    self.rev_take_box_from_back()
                    self.rev_take_box_from_back()
                    flag = False
                    fork_counter +=1
                    self.halt()
                    break
            

    def handle_forks_to_yellow_cube(self):
        fork_counter = 0
        num_of_box = 0
        flag = True
        flag2 = True
        stage_5_completed = False
        stop_counter = 0  # عداد للتحكم في الإيقاف
        stop_limit = 305
        while self.step(self.timestep) != -1:
        # Step 1: Line follow until a fork is detected
            print("Line following until a fork is detected...")
            if(flag):
                self.line_follow()

            if fork_counter == 0:
                if self.detect_fork():
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.detect_fork():
                        self.handle_fork_left()
                        self.step(self.timestep)
                    fork_counter += 1
                
            
                
            elif fork_counter == 1  :
                if self.detect_fork():
                    print("Second fork detected. Handling with handle_fork_left().")
                    while self.detect_fork():
                        self.handle_fork_left()
                        self.step(self.timestep)
                    print("Reached the yellow cube. Performing the yellow job.")
                    fork_counter += 1
                    print("forkkkkkkkkkkkkkkkkkkk",fork_counter)
                    
            elif fork_counter == 2 :
                print(666666666666666666666666666666666666666666666)
                print("Detecting blue floor to place cubes...")
                distance_x, distance_y, distance_z = self.detect_yelow_floor()

# Check if the robot is close to the blue floor
                if distance_x <= 0.2:
                    print("yelow floor detected. Placing the cube...")
                    self.halt()  

                    self.rev_take_box_from_back()  
                    print("Cube placed on the yelow floor.")

                    print("Moving to the next stage...")
                    fork_counter += 1
                    print("Fork counter incremented. New value:", fork_counter)
        
            
            elif fork_counter == 3 :
                print("the fork count is===",fork_counter)
                
                print("Starting line following after 180-degree turn...")
                flag = False
                self.handle_minus_180_degree_turn_yellow()
                # self.halt()
                # self.handle_180_degree_turn()
                flag = True
                self.move_forward(14.81)
                fork_counter += 1
                transition_counter = 0  # إعادة تعيين العداد
                
                print("end 180 rotate")

            elif fork_counter == 4:
            # زيادة عداد الانتقال خلال هذه المرحلة
                transition_counter += 0.01
                print(f"Transition counter: {transition_counter}")

                # تحقق من انتهاء الانتقال
                if transition_counter >= 4.50:  # يمكنك تعديل القيمة حسب الحاجة
                    print("Transition to state 5 allowed.")
                    fork_counter += 1
                    continue  # الانتقال إلى الحالة الخامسة

                if self.is_line_right():
                    print("First fork detected. Handling with handle_fork_right().")
                    while self.is_line_right():
                        self.handle_fork_right()
                        self.step(self.timestep)

                    print("forrrrrrrrrrrrrrrrrrrrrrrrrrrkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
            
            elif fork_counter ==5:
                if self.is_line_right():
                    # flag2 = False
                    print("second fork detected. Handling with handle_fork_left().")
                    while self.is_line_right():
                        self.handle_fork_right()
                        self.step(self.timestep)
                    stage_5_completed = True  # هنا تحدد أنك انتهيت من المرحلة 5
                if stage_5_completed:
                    print("Stage 5 completed. Moving to stage 6...")
                    fork_counter += 1  # الانتقال إلى المرحلة 6
                    continue  # الانتقال إلى المرحلة التالية
            
            
            elif fork_counter ==6:
                print("reverse the box from back to the wall")
                stop_counter += 1  # زيادة العداد في كل خطوة
                print(f"Stop counter: {stop_counter}")

                if stop_counter >= stop_limit:
                    print("Stop limit reached. Halting the robot.")
                    self.halt()
                    # self.rev_take_box_from_back()
                    # self.rev_take_box_from_back()
                    flag = False
                    fork_counter +=1
                    self.halt()
                    break
            
    
    
    
    def handle_forks_to_red_cube(self):
        fork_counter = 0
        num_of_box = 0
        flag = True
        flag2 = True
        stage_5_completed = False
        stop_counter = 0  # عداد للتحكم في الإيقاف
        stop_limit = 600
        while self.step(self.timestep) != -1:
        # Step 1: Line follow until a fork is detected
            print("Line following until a fork is detected...")
            if(flag):
                self.line_follow()

            if fork_counter == 0:
                if self.detect_fork():
                    print("First fork detected. Handling with handle_fork_right().")
                    while self.detect_fork():
                        self.handle_fork_right()
                        self.step(self.timestep)
                    fork_counter += 1
                
            
                
            elif fork_counter == 1  :
                if self.detect_fork():
                    print("Second fork detected. Handling with handle_fork_right().")
                    while self.detect_fork():
                        self.handle_fork_right()
                        self.step(self.timestep)
                        # Exit the loop after the second fork
                    print("Reached the red cube. Performing the red job.")
                    fork_counter += 1
                    print("forkkkkkkkkkkkkkkkkkkk",fork_counter)
                    
            elif fork_counter == 2 :
                print(666666666666666666666666666666666666666666666)
                print("Detecting blue floor to place cubes...")
                distance_x, distance_y, distance_z = self.detect_red_floor()

                # Check if the robot is close to the blue floor
                if distance_x <= 0.2:  
                    print("yelow floor detected. Placing the cube...")
                    self.halt()  

                    self.rev_take_box_from_back()  
                    print("Cube placed on the yelow floor.")

                    print("Moving to the next stage...")
                    fork_counter += 1
                    print("Fork counter incremented. New value:", fork_counter)
        
            
            elif fork_counter == 3 :
                print("the fork count is===",fork_counter)
                
                print("Starting line following after 180-degree turn...")
                flag = False
                self.handle_minus_180_degree_turn()
                # self.halt()
                # self.handle_180_degree_turn()
                flag = True
                self.move_forward(14.81)
                fork_counter += 1
                transition_counter = 0  # إعادة تعيين العداد
                
                print("end 180 rotate")
            
            
            
            
            
            elif fork_counter == 4:
            # زيادة عداد الانتقال خلال هذه المرحلة
                transition_counter += 0.01
                print(f"Transition counter: {transition_counter}")

                # تحقق من انتهاء الانتقال
                if transition_counter >= 4.50:  # يمكنك تعديل القيمة حسب الحاجة
                    print("Transition to state 5 allowed.")
                    fork_counter += 1
                    continue  # الانتقال إلى الحالة الخامسة

                if self.is_line_left():
                    print("First fork detected. Handling with handle_fork_right().")
                    while self.is_line_left():
                        self.handle_fork_left()
                        self.step(self.timestep)

                    print("forrrrrrrrrrrrrrrrrrrrrrrrrrrkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
            
            elif fork_counter ==5:
                if self.is_line_left():
                    # flag2 = False
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.is_line_left():
                        self.handle_fork_left()
                        self.step(self.timestep)
                    stage_5_completed = True  # هنا تحدد أنك انتهيت من المرحلة 5
                if stage_5_completed:
                    print("Stage 5 completed. Moving to stage 6...")
                    fork_counter += 1  # الانتقال إلى المرحلة 6
                    continue  # الانتقال إلى المرحلة التالية
                
            elif fork_counter ==6:
                print("reverse the box from back to the wall")
                stop_counter += 1  # زيادة العداد في كل خطوة
                print(f"Stop counter: {stop_counter}")

                if stop_counter >= stop_limit:
                    print("Stop limit reached. Halting the robot.")
                    self.halt()
                    # self.rev_take_box_from_back()
                    # self.rev_take_box_from_back()
                    flag = False
                    fork_counter +=1
                    self.halt()
                    break
            
            


        
    def detect_blue_floor(self ):
        objects = self.camera.getRecognitionObjects()
        distance_x=10000
        distance_y=10000
        distance_z=10000

        for obj in objects:
            model_label = obj.getModel() 
            # if model_label != 'box' and model_label.lower() != 'rectangular panel':
            print(f"Detected object model: {model_label}")
            if model_label == 'blueee':
                
                print("1")
                distance_x = abs(obj.getPosition()[0] )         
                distance_y = abs(obj.getPosition()[1] ) 
                distance_z = abs(obj.getPosition()[2] ) 

                print(f"distance_x is :  {distance_x}\n") 
                print(f"distance_y is :  {distance_y}\n") 
                print(f"distance_z is :  {distance_z}\n") 

                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
            # Define thresholds for red color detection
                if red == 0 and green == 0 and blue == 1:
                    # if distance_x <= 0.08 and distance_y <= 0.025 and distance_z  <= 0.0037 :
                        print("2")
                        return distance_x , distance_y ,distance_z
        return distance_x , distance_y ,distance_z

    def detect_yelow_floor(self ):
        objects = self.camera.getRecognitionObjects()
        distance_x=10000
        distance_y=10000
        distance_z=10000

        for obj in objects:
            model_label = obj.getModel() 
            # if model_label != 'box' and model_label.lower() != 'rectangular panel':
            print(f"Detected object model: {model_label}")
            if model_label == 'yelow':
                
                print("1")
                distance_x = abs(obj.getPosition()[0] )         
                distance_y = abs(obj.getPosition()[1] ) 
                distance_z = abs(obj.getPosition()[2] ) 

                print(f"distance_x is :  {distance_x}\n") 
                print(f"distance_y is :  {distance_y}\n") 
                print(f"distance_z is :  {distance_z}\n") 

                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
            # Define thresholds for red color detection
                if red == 1 and green == 1 and blue == 0:
                    # if distance_x <= 0.08 and distance_y <= 0.025 and distance_z  <= 0.0037 :
                        print("2")
                        return distance_x , distance_y ,distance_z
        return distance_x , distance_y ,distance_z    

    def detect_red_floor(self ):
        objects = self.camera.getRecognitionObjects()
        distance_x=10000
        distance_y=10000
        distance_z=10000

        for obj in objects:
            model_label = obj.getModel() 
            # if model_label != 'box' and model_label.lower() != 'rectangular panel':
            print(f"Detected object model: {model_label}")
            if model_label == 'red':
                
                print("1")
                distance_x = abs(obj.getPosition()[0] )         
                distance_y = abs(obj.getPosition()[1] ) 
                distance_z = abs(obj.getPosition()[2] ) 

                print(f"distance_x is :  {distance_x}\n") 
                print(f"distance_y is :  {distance_y}\n") 
                print(f"distance_z is :  {distance_z}\n") 

                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
            # Define thresholds for red color detection
                if red == 1 and green == 0 and blue == 0:
                    # if distance_x <= 0.08 and distance_y <= 0.025 and distance_z  <= 0.0037 :
                        print("2")
                        return distance_x , distance_y ,distance_z
        return distance_x , distance_y ,distance_z        












    def handle_forks_to_green_cube(self):
        fork_counter = 0
        num_of_box = 0
        flag = True
        flag2 = True
        stage_5_completed = 0
        stop_counter = 0  # عداد للتحكم في الإيقاف
        stop_limit = 305
        while self.step(self.timestep) != -1:
        # Step 1: Line follow until a fork is detected
            print("Line following until a fork is detected...")
            if(flag):
                self.line_follow()

            if fork_counter == 0:
                if self.detect_fork():
                    print("First fork detected. Handling with handle_fork_right().")
                    while self.detect_fork():
                        self.handle_fork_right()
                        self.step(self.timestep)
                    fork_counter += 1
                
            
                
            elif fork_counter == 1  :
                if self.detect_fork():
                    print("Second fork detected. Handling with handle_fork_left().")
                    while self.detect_fork():
                        self.handle_fork_left()
                        self.step(self.timestep)
                        # Exit the loop after the second fork
                    print("Reached the green cube. Performing the green job.")
                    fork_counter += 1
                    print("forkkkkkkkkkkkkkkkkkkk",fork_counter)
                    
            elif fork_counter == 2 :
                print(666666666666666666666666666666666666666666666)
                action, position, distance = self.detect_box()
                print("catchessssssssssssssssssssssss")
                if action == 'go_and_catch':
                    # Check if the detected box color is green
                    if self.detected_box_color == (0, 1, 0)  :  # green color
                        print("green box detected. Approaching the box...")
                        self.approach_box(position, distance)
                        num_of_box += 1
                        print("number of box is",num_of_box)
                        if num_of_box == 4:
                            print("boxessss is 4 ")
                            fork_counter += 1
                            print("forkkkkkk  green finish",fork_counter)
            
            
            elif fork_counter == 3 :
                print("the fork count is===",fork_counter)
                print("the fork count is===",fork_counter)
                
                print("Starting line following after 180-degree turn...")
                flag = False
                self.handle_180_degree_turn_green()
                # self.halt()
                # self.handle_180_degree_turn()
                flag = True
                self.move_forward(14.81)
                fork_counter += 1
                transition_counter = 0  # إعادة تعيين العداد
                
                print("end 180 rotate")
                
                
            elif fork_counter == 4:
            # زيادة عداد الانتقال خلال هذه المرحلة
                transition_counter += 0.01
                print(f"Transition counter: {transition_counter}")

                # تحقق من انتهاء الانتقال
                if transition_counter >= 4.50:  # يمكنك تعديل القيمة حسب الحاجة
                    print("Transition to state 5 allowed.")
                    fork_counter += 1
                    continue  # الانتقال إلى الحالة الخامسة

                if self.is_line_right():
                    print("First fork detected. Handling with handle_fork_right().")
                    while self.is_line_right():
                        self.handle_fork_right()
                        self.step(self.timestep)

                    print("forrrrrrrrrrrrrrrrrrrrrrrrrrrkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
            
            elif fork_counter ==5:
                
                stage_5_completed += 0.01
                print(f"stage_5_completed: {stage_5_completed} ")
                
                if stage_5_completed >=4.50 :
                    print("Stage 5 completed. Moving to stage 6...")
                    fork_counter += 1  # الانتقال إلى المرحلة 6
                    continue  # الانتقال إلى المرحلة التالية
            
                
                if self.is_line_left():
                    # flag2 = False
                    print("First fork detected. Handling with handle_fork_left().")
                    while self.is_line_left():
                        self.handle_fork_left()
                        self.step(self.timestep)
                
            elif fork_counter ==6:
                print("reverse the box from back to the wall")
                stop_counter += 1  # زيادة العداد في كل خطوة
                print(f"Stop counter: {stop_counter}")

                if stop_counter >= stop_limit:
                    print("Stop limit reached. Halting the robot.")
                    self.halt()
                    # self.rev_take_box_from_back()
                    # self.rev_take_box_from_back()
                    flag = False
                    fork_counter +=1
                    self.halt()
                    break        
    
    
    

    def move_robot(self):
        while self.step(self.timestep) != -1:
            action, position, distance = self.detect_box()
            distance_x , distance_y ,distance_z = self.detect_blue_floor()
            
            print("Start Take Direction222222222222222")
            if action == 'change_direction':
                print("change_direction1")
            elif action == 'go_and_catch' and distance > 0.04:
                print("go_and_catch22222222222222")
                if  (self.detected_box_color[1] != 1):
                        print("pickup_state2222222222222 ====",self.pickup_state)
                        self.approach_box(position, distance)
                        print(f"distance_x :{distance_x} distance_y:{distance_y} distance_z:{distance_z}" )
                        

            elif distance_x <= 0.2:
                            self.halt()
                            self.rev_take_box_from_back()
                            self.rev_take_box_from_back()
                            break
            else:
                self.line_follow()
                #   self.rotate_180_cw()
                
                if self.detect_fork():
                    self.handle_fork()
                
                # if self.is_line_right():
                #     self.handle_fork_right()
                
                # elif self.is_line_left():
                #     self.handle_fork_left()
                
                # if not self.is_path_right() and not self.is_path_left() and self.is_wall_corner():
                    # print("turn_around")
                    # self.rotate(math.pi)  # Rotate 180 degrees
                # if self.is_wall_corner():
                #     print("corner_turn_left")
                #     self.rotate(-math.pi / 2)  # Rotate 90 degrees to the left
                #     self.move_forward(self.MOVEMENT_VELOCITY)
                # elif self.is_path_right():
                #     print("turn_right")
                #     self.rotate(-math.pi / 2)  # Rotate 90 degrees to the left
                #     self.move_forward(self.MOVEMENT_VELOCITY)
                # elif self.is_path_forward():
                #     print("move_forward")
                #     self.move_forward(self.MOVEMENT_VELOCITY)
                # elif self.is_path_left():
                #     print("turn_left")
                #     self.rotate(math.pi / 2)  # Rotate 90 degrees to the right
                #     self.move_forward(self.MOVEMENT_VELOCITY)
                # else:
                #     print("turn_around")
                #     self.rotate(math.pi)  # Rotate 180 degrees



    # def loop(self):
    #     """Main loop for the robot."""
    #     while self.step(self.timestep) != -1:
    #         # Regular line following
    #         self.line_follow()
    #         self.detect_box()
    #         # Detect and handle fork
    #         if self.detect_fork():
    #             self.handle_fork()
            
    def loop(self):
        self.turn_completed = False
        self.picked_up_cube = False  # Track if the robot has picked up a cube

        while self.step(self.timestep) != -1:
            action, position, distance = self.detect_box()
            print(action)

            if action == 'no_box' and not self.picked_up_cube:
                print("No cubes detected, waiting or searching...")
                self.halt()  # Keep  waiting for a cube
                continue

            if action == 'no_box' and self.picked_up_cube and not self.turn_completed:
                print("All cubes processed. Performing 180-degree turn...")
                self.handle_180_degree_turn_start()
                self.turn_completed = True

            elif action == 'go_and_catch':
                print("Cube detected! Picking up...")
                
                self.approach_boxwall(position, distance)
                self.picked_up_cube = True
                self.turn_completed = False

            elif self.turn_completed:
                print("Turn completed. Resuming  the jobs ...")
                while self.step(self.timestep) != -1:
                    # Step 1: Detect and process colors in the matrix
                    target_color = self.detect_and_process_colors()

                    # Step 2: Handle the target cube based on its color
                    if target_color == (0, 0, 1):  # Blue color
                        print("Blue matrix detected. Handling blue cube...")
                        self.handle_forks_to_blue_cube()
                    elif target_color == (1, 1, 0):  # Yellow color
                        print("Yellow matrix detected. Handling yellow cube...")
                        self.handle_forks_to_yellow_cube()
                    elif target_color == (1, 0, 0):  # Red color
                        print("Red matrix detected. Handling red cube...")
                        self.handle_forks_to_red_cube()
                    elif target_color == (0, 1, 0):  # Green color
                        print("Green matrix detected. Handling green cube...")
                        self.handle_forks_to_green_cube()

           

robot = RobotController()
robot.loop()

