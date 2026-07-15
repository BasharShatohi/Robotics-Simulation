from controller import Robot
import math

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        # Setup for wheels
        self.timestep = int(self.getBasicTimeStep())
        self.wheels = [self.getDevice("wheel" + str(i)) for i in range(1, 5)]
        self.detected_box_color = None
        self.colors_in_order = None
        self.is_picking_up = False
        self.pickup_state = 'align'
        self.rotation_completed= False
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

        self.max_velocity = 14.81
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
        self.base_speed =10.28
        self.turn_duration = 1500  # Placeholder: the duration of a turn (to be calibrated)
        self.turn_around_duration = 3000  # Placeholder: the duration of a 180-degree turn (to be calibrated)
        self.step(self.timestep)

    def move_forward(self, speed):
        self.set_motors_velocity(speed, speed, speed, speed)


    def backward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(-7.0) # maxVelocity = 14.81
        self.step(time * self.timestep)
    
    def minus_backward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(7.0) # maxVelocity = 14.81
        self.step(time * self.timestep)
    
    
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
        


    def rotate(self, angle):
        # Rotate the robot by a specified angle (in radians)
        rotation_speed = self.turn_speed if angle > 0 else -self.turn_speed
        rotation_time = abs(angle / self.turn_speed)
        self.set_wheel_speeds(rotation_speed, -rotation_speed, rotation_speed, -rotation_speed)
        
        end_time = self.getTime() + rotation_time
        while self.getTime() < end_time:
            if self.step(self.timestep) == -1:
                break


    def rotate_180_cw(self):


        # Total duration for the turn (adjust based on testing)
        turn_duration = 3000  # in ms
        num_steps = turn_duration // self.timestep
        rotation_time = abs(math.pi / self.turn_speed)

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            self.turn_cw(speed)
            end_time = self.getTime() + rotation_time
        while self.getTime() < end_time:
            if self.step(self.timestep) == -1:
                break    
        # Reset PID controller
        self.last_error = 0





    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)
    
    def halt(self):
        for wheel in self.wheels:
            wheel.setVelocity(0)

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
            print(f"T-shaped fork detected! ")
            return True
        else:
            return False


    
    def handle_fork_right(self):
        """Handle the fork by performing a smooth right turn."""
        print("Handling fork by performing a smooth right turn")

        turn_duration = 1500  # in ms
        num_steps = turn_duration // self.timestep
        

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            self.turn_cw(speed)
            self.step(self.timestep )

        print(" turn complete")
        
        

        #     # Reset wheel speeds to standard line-following speed
        # self.front_right_wheel.setVelocity(self.movement_velocity / 2)
        # self.back_right_wheel.setVelocity(self.movement_velocity / 2)
        # self.front_left_wheel.setVelocity(self.movement_velocity / 2)
        # self.back_left_wheel.setVelocity(self.movement_velocity / 2)
       
        # Reset PID controller
        self.last_error = 0




    def turn_90(self):
            """Handle the fork by performing a smooth right turn."""
            print("Handling fork by performing a smooth right turn")

            turn_duration = 5800# in ms
            num_steps = turn_duration // self.timestep
            

            for step in range(num_steps):
                factor = math.sin(math.pi * step / num_steps)

                speed = 13.97 * factor
                self.front_left_wheel.setVelocity(self.max_velocity)
                self.back_left_wheel.setVelocity(self.max_velocity)
                self.front_right_wheel.setVelocity(-self.max_velocity)
                self.back_right_wheel.setVelocity(-self.max_velocity)
                self.step(self.timestep )
                self.halt()

            print(" turn complete")

    def turn_minus_90(self):
        """Handle the fork by performing a smooth right turn."""
        print("Handling fork by performing a smooth right turn")

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
    
    def handle_minus_180_degree_turn_yellow(self):
        """Perform a -180-degree turn by calling handle_fork_right twice."""
        print("Starting -180-degree turn...")

        # First 90-degree turn
        self.turn_minus_90()
        self.minus_backward(26)
        # self.step(7000)

        # Second 90-degree turn
        self.turn_minus_90()

        print("-180-degree turn complete.")
    
    

        
    def handle_fork_left(self):
        """Handle the fork by performing a smooth left turn."""

        turn_duration = 1500  # in ms
        num_steps = turn_duration // self.timestep

        for step in range(num_steps):
            factor = math.sin(math.pi * step / num_steps)

            speed = self.base_speed * factor 
            self.move_left(speed)
            self.step(self.timestep )

        # Reset PID controller
        self.last_error = 0
        


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

    def perform_job_for_color(self, color):
        """Perform a specific job based on the detected color."""
        red, green, blue = color

        if red == 0 and green == 0 and blue == 1:
            print("Blue matrix_plane detected. Performing Job (1)")
            # self.handle_forks_to_blue_cube()
        
        elif red == 1 and green == 1 and blue == 0:
            print("Yellow matrix_plane detected. Performing Job (1)")
            # self.handle_forks_to_yellow_cube()
            
        elif red == 1 and green == 0 and blue == 0:
            print("Red matrix_plane detected. Performing Job (1)")
            # self.handle_forks_to_red_cube()
        
        elif red == 0 and green == 1 and blue == 0:
            print("Green matrix_plane detected. Performing Job (1)")
            # self.handle_forks_to_green_cube()
        
        
        else:
            print("Unknown color. No job performed.")



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
                action, position, distance = self.detect_box()
                print("catchessssssssssssssssssssssss")
                if action == 'go_and_catch':
                    # Check if the detected box color is blue
                    if self.detected_box_color == (0, 0, 1) and (self.detected_box_color[1] != 1) :  # Blue color
                        print("Blue box detected. Approaching the box...")
                        self.approach_box(position, distance)
                        num_of_box += 1
                        print("number of box is",num_of_box)
                        if num_of_box == 4:
                            print("boxessss is 4 ")
                            fork_counter += 1
                            print("forkkkkkk  blue finish",fork_counter)
            
            
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
                    # self.rev_take_box_from_back()
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
                action, position, distance = self.detect_box()
                print("catchessssssssssssssssssssssss")
                if action == 'go_and_catch':
                    # Check if the detected box color is yellow
                    if self.detected_box_color == (1, 1, 0)  :  # yellow color
                        print("yellow box detected. Approaching the box...")
                        self.approach_box(position, distance)
                        num_of_box += 1
                        print("number of box is",num_of_box)
                        if num_of_box == 4:
                            print("boxessss is 4 ")
                            fork_counter += 1
                            print("forkkkkkk  yellow finish",fork_counter)
            
            
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
                action, position, distance = self.detect_box()
                print("catchessssssssssssssssssssssss")
                if action == 'go_and_catch':
                    # Check if the detected box color is red
                    if self.detected_box_color == (1, 0, 0)  :  # red color
                        print("red box detected. Approaching the box...")
                        self.approach_box(position, distance)
                        num_of_box += 1
                        print("number of box is",num_of_box)
                        if num_of_box == 4:
                            print("boxessss is 4 ")
                            fork_counter += 1
                            print("forkkkkkk  red finish",fork_counter)
            
            
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

                if distance <= 0.15:
                    return 'go_and_catch', position, distance

        return 'no_box', 0, float("inf")
                
                
    
    
    def approach_box(self, position, distance):
        print(f"the Distance is {distance}")
        pickup_distance = 0.17
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
    
    
    def perform_pick_and_place(self):
        self.halt()
        # self.step(100 * self.timestep)  # Wait a moment
        
        self.pick_up()
        # self.pick_up_boxwall()
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
        self.armMotors[2].setPosition(-1)
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
            self.armMotors[0].setPosition(-2.9)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-1)
            self.armMotors[2].setPosition(-1.7)
            self.armMotors[4].setPosition(2.9)
    
        elif self.COUNTER_PICKK_UP == 4 :
            self.armMotors[0].setPosition(0)
            self.armMotors[1].setPosition(1.2)
            self.armMotors[2].setPosition(-0.125)
            self.armMotors[3].setPosition(1.5)
            self.armMotors[4].setPosition(0)
    
    
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
    
    
    
    
        # while not self.detect_fork():  # Detect the first fork
        #     self.line_follow()
        #     self.step(3000)

        # Step 2: Handle the first fork (turn left)
        # print("First fork detected. Handling with handle_fork_left().")
        # self.handle_fork_left()
        # self.step(7000)

        # # Step 3: Line follow until the second fork
        # print("Line following until the second fork...")
        # while not self.detect_fork():  # Detect the second fork
        #     self.line_follow()
        #     self.step(3000)

        # # Step 4: Handle the second fork (turn right)
        # print("Second fork detected. Handling with handle_fork_right().")
        # self.halt()
        # self.step(3000)
        # self.handle_fork_right()
        # self.step(7000)
        

        # # Step 5: Reach the blue cube and perform the blue job
        # print("Reached the blue cube. Performing the blue job.")
        # Stop for 2 seconds


    def is_job_complete(self):
        """Check if the current job is complete."""
        # Example: Check if the robot has stopped moving
        return False
    
    

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
            self.armMotors[1].setPosition(1.2)
            self.armMotors[2].setPosition(0.19)
            self.armMotors[3].setPosition(1.05)
            self.armMotors[4].setPosition(0)
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
        elif self.COUNTER_REV_PICKK_UP == 2:
            self.armMotors[0].setPosition(-2.90)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-0.7)
            self.armMotors[2].setPosition(-2)
            self.armMotors[4].setPosition(2.9)
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



    def rev_drop(self):
        # Move arm down
        self.armMotors[3].setPosition(-1.14)
        self.armMotors[2].setPosition(-1)
        self.armMotors[1].setPosition(-1.13)
        self.armMotors[0].setPosition(-0.17)
        self.step(100 * self.timestep)


        # Open gripper.
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)


    def rev_fold_arms(self):
        self.armMotors[3].setPosition(0)
        self.step(50 * self.timestep)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[0].setPosition(0)
        










    def move_robot(self):
        while self.step(self.timestep) != -1:
            action, position, distance = self.detect_box()
            # distance_x , distance_y ,distance_z = self.detect_blue_floor()
            
            print("Start Take Direction222222222222222")
            
            if action == 'go_and_catch' and distance > 0.04:
                print("go_and_catch22222222222222")
                if  (self.detected_box_color[1] != 1):
                        print("pickup_state2222222222222 ====",self.pickup_state)
                        self.approach_box(position, distance)
                        print(f"distance_x :{distance_x} distance_y:{distance_y} distance_z:{distance_z}" )

            # elif distance_x <= 0.2:
            #                 self.halt()
            #                 self.rev_take_box_from_back()
            #                 self.rev_take_box_from_back()
            #                 break

            
            else:
                self.line_follow()
                #   self.rotate_180_cw()
                
                if self.detect_fork():
                    self.handle_fork()
    
    
    
    
    
    
    
    def loop(self):
        """Main loop for the robot."""
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

            # Step 3: After reaching the target cube, detect and handle boxes of the same color
            # while self.step(self.timestep) != -1:
            #     action, position, distance = self.detect_box()

            #     if action == 'go_and_catch':
            #         # Check if the detected box color matches the target color
            #         if self.detected_box_color == target_color:
            #             print(f"{target_color} box detected. Approaching the box...")
            #             self.approach_box(position, distance)
            #             break  # Exit the loop after handling the box
            #         else:
            #             print(f"Detected box color does not match the target color. Ignoring...")
            #             continue  # Continue searching for the correct box

    

    # def loop(self):
    #     """Main loop for the robot."""
    #     while self.step(self.timestep) != -1:
    #         # self.line_follow()
    #         self.detect_and_process_colors()
            
    #         # Step 2: If the first color is blue, handle the blue cube
    #         if self.detected_box_color == (0, 0, 1):  # Blue color
    #             print("Blue matrix detected. Handling blue cube...")
    #             self.handle_forks_to_blue_cube()

    #             # Step 3: After reaching the blue cube, detect and handle boxes
    #             while self.step(self.timestep) != -1:
    #                 action, position, distance = self.detect_box()

    #                 if action == 'go_and_catch':
    #                     print("Box detected. Approaching the box...")
    #                     self.approach_box(position, distance)
    #                     break  # Exit the loop after handling the box
                
                
            
    #         else:
    #             if self.line_follow() ==  True: # If line_follow returns True, gate marker is detected
    #                 print("hiiiiiiiiiiiiiiiiiiiiiiiiii")
    #                 self.move_robot()
    #                 # print("Start Navigate Maze")
    #                 # if self.detect_fork():
    #                 #     self.handle_fork()
    #                 # self.navigate_maze()
    #                 break
            
            
            # Detect and process colors


        
    # def detect_box(self):
    #  """Detect the closest box in the camera's recognition objects."""
    #  objects = self.camera.getRecognitionObjects()
    #  print(f"Number of objects detected: {len(objects)}")
    #
    #  for obj in objects:
    #      model_label = obj.getModel()
    #      print(f"Detected object model: {model_label}") 
#
    #      colors = obj.getColors()
    #      red, green, blue = colors[0], colors[1], colors[2]
    #      current_color = (red, green, blue)
    #      self.detected_box_color = current_color
    #      # print(f"Detected box: {self.detected_box_color}") 
    #      # print(f"Box detected with color: R={red:.2f}, G={green:.2f}, B={blue:.2f}")
    #
    #      if model_label == 'box':
    #          x, y = obj.getPositionOnImage()
    #          image_width = self.camera.getWidth()
    #          position = (x - (image_width / 2)) / (image_width / 2)  # Normalized [-1, 1]
    #          distance = abs(obj.getPosition()[0])
    #
    #          if green == 1 and red == 0 and blue == 0:
    #              print(f"Green box detected at distance {distance}")
    #              if distance <= 0.4:
    #                  return 'change_direction', position, distance
    #
    #          elif green != 0 and red != 0 and blue == 0:  
    #              print(f"Yellow box detected at distance {distance}")
    #              if distance <= 0.4:
    #                  return 'change_direction', position, distance
    #
    #          elif red == 1 and green == 0 and blue == 0:
    #              print(f"Red box detected at distance {distance}")
    #              if distance <= 0.4:
    #                  return 'change_direction', position, distance
    #
    #          elif red == 0 and green == 0 and blue == 1:
    #              print(f"Blue box detected at distance {distance}")
    #              if distance <= 0.4:
    #                  return 'change_direction', position, distance
    #
    #          else:
    #              if distance <= 0.35:
    #                  return 'go_and_catch', position, distance
    #
    #  return 'no_box', 0, float("inf")
#
#
#
#
    #
    # def loop(self):
    #     while self.step(self.timestep) != -1:
    #         self.line_follow()

            # if not self.rotation_completed:
            #     self.rotate_180_cw()
            #     self.rotation_completed = True  # Set the flag to True after rotation
                
            # Regular line following
            # self.line_follow()
            # self.detect_box()

            # Detect and handle fork
            # if self.is_line_left():
            #     self.handle_fork_left()
            #  if self.is_line_right():
            #     self.rotate_180_degrees()
            # elif self.detect_fork():
            #      self.handle_fork_left()
                
                    


                #self.detect_fork():
                # self.handle_fork_left()
                #self.handle_fork_right

# Create and start the robot controller
robot = RobotController()
robot.loop()

