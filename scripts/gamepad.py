import pygame

class Commands:
    """MiniClass for tracking sit and stand commands."""
    def __init__(self):
        self.sit = False
        self.stand = False

class GameController:
    DEADZONE = 0.25  # Thumbstick deadzone

    # Button mappings (for Xbox controller)
    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_RB = 10   # Right bumper
    BUTTON_DPAD_UP = 11    # D-pad up
    BUTTON_DPAD_DOWN = 12  # D-pad down
    
    # Trigger axes
    AXIS_RT = 4  # Right trigger
    AXIS_LT = 5  # Left trigger

    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick connected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.mode = 'walk'  # Default mode: 'walk', 'stand'
        
        # Commands instance
        self.commands = Commands()
        
        # Debug mode - set to True to see all button presses
        self.debug_buttons = False

        # Walking velocities
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_rot = 0.0

        # Standing angles and height
        self.yaw = 0.0
        self.pitch = 0.0
        self.height = 0.0

        # Trigger values
        self.rt_value = 0.0
        self.lt_value = 0.0
        
        # RB hold state for stand mode
        self.rb_held = False

        print(f"[GameController] Joystick initialized: {self.joystick.get_name()}")
        
        if self.debug_buttons:
            self._print_controller_info()

    def _print_controller_info(self):
        """Print controller information and button mapping for debug purposes."""
        print("\n[DEBUG] Controller Information:")
        print(f"  Name: {self.joystick.get_name()}")
        print(f"  Axes: {self.joystick.get_numaxes()}")
        print(f"  Buttons: {self.joystick.get_numbuttons()}")
        print(f"  Hats: {self.joystick.get_numhats()}")
        print(f"  Balls: {self.joystick.get_numballs()}")

    def _reset_velocities_and_angles(self):
        """Reset all velocities, angles, and height when changing modes."""
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_rot = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.height = 0.0

    def process_events(self):
        # Update command states based on current button presses
        self.commands.sit = self.joystick.get_button(self.BUTTON_X)
        self.commands.stand = self.joystick.get_button(self.BUTTON_A)
        
        for event in pygame.event.get():
            # Debug: Show all pygame events
            if self.debug_buttons:
                if event.type == pygame.JOYAXISMOTION:
                    # Only show axis motion if it's significant (to reduce spam)
                    if abs(event.value) > 0.1:
                        axis_names = {0: "Left-X", 1: "Left-Y", 2: "Right-X", 3: "Right-Y", 4: "RT", 5: "LT"}
                        axis_name = axis_names.get(event.axis, f"Axis-{event.axis}")
                        print(f"[DEBUG] {axis_name}: {event.value:.3f}")
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"[DEBUG] BUTTON {event.button} PRESSED")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"[DEBUG] BUTTON {event.button} RELEASED")
                elif event.type == pygame.JOYHATMOTION:
                    hat_directions = {(0, 1): "UP", (0, -1): "DOWN", (1, 0): "RIGHT", (-1, 0): "LEFT", (0, 0): "CENTER"}
                    direction = hat_directions.get(event.value, f"({event.value[0]}, {event.value[1]})")
                    print(f"[DEBUG] D-PAD {event.hat}: {direction}")
                elif event.type == pygame.JOYBALLMOTION:
                    print(f"[DEBUG] BALL {event.ball}: {event.rel}")
                elif event.type not in [pygame.QUIT, pygame.NOEVENT]:
                    pass
            
            if event.type == pygame.QUIT:
                return False

            if event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = event.value

                if abs(value) < self.DEADZONE:
                    value = 0.0  # Deadzone

                # Left stick vertical
                if axis == 1:  
                    if self.mode == 'walk':
                        self.v_x = -value
                    elif self.mode == 'stand':
                        # In stand mode, left joystick up/down controls height scaled by 0.15
                        self.height = -value * 0.15  # Scale height

                # Left stick horizontal
                elif axis == 0:  
                    if self.mode == 'walk':
                        self.v_y = value

                # Right stick horizontal
                elif axis == 2:  
                    if self.mode == 'walk':
                        self.v_rot = value * 1.0  # Rotation speed
                    elif self.mode == 'stand':
                        self.yaw = -value * 0.5  # Scale yaw

                # Right stick vertical
                elif axis == 3:  
                    if self.mode == 'stand':
                        self.pitch = value * 0.5  # Scale pitch

                # Right trigger
                elif axis == self.AXIS_RT:  
                    self.rt_value = max(0.0, value)  # Ensure positive value
                    if self.mode == 'walk':
                        # Calculate pitch as difference: RT - LT
                        self.pitch = (self.rt_value - self.lt_value) * 0.5

                # Left trigger
                elif axis == self.AXIS_LT:  
                    self.lt_value = max(0.0, value)  # Ensure positive value
                    if self.mode == 'walk':
                        # Calculate pitch as difference: RT - LT
                        self.pitch = (self.rt_value - self.lt_value) * 0.5

            if event.type == pygame.JOYBUTTONDOWN:
                button = event.button

                if button == self.BUTTON_A:
                    # A switches to walk mode
                    if self.mode != 'walk':
                        self.mode = 'walk'
                        self._reset_velocities_and_angles()
                        print("[GameController] Switched to Walk mode (A pressed)")

                elif button == self.BUTTON_B:
                    # B switches to stand mode
                    if self.mode != 'stand':
                        self.mode = 'stand'
                        self._reset_velocities_and_angles()
                        print("[GameController] Switched to Stand mode (B pressed)")

                elif button == self.BUTTON_RB:
                    # RB hold activates stand mode
                    self.rb_held = True
                    if self.mode != 'stand':
                        self.mode = 'stand'
                        self._reset_velocities_and_angles()
                        print("[GameController] Switched to Stand mode (RB held)")

                elif button == self.BUTTON_DPAD_UP:
                    # D-pad up controls height in walk mode
                    if self.mode == 'walk':
                        self.height = min(0.25, self.height + 0.05)
                        print(f"[GameController] Height increased to {round(self.height, 2)}")

                elif button == self.BUTTON_DPAD_DOWN:
                    # D-pad down controls height in walk mode
                    if self.mode == 'walk':
                        self.height = max(-0.25, self.height - 0.05)
                        print(f"[GameController] Height decreased to {round(self.height, 2)}")

            if event.type == pygame.JOYBUTTONUP:
                button = event.button
                
                if button == self.BUTTON_RB:
                    # RB release returns to walk mode if it was held for stand mode
                    self.rb_held = False
                    if self.mode == 'stand':
                        self.mode = 'walk'
                        self._reset_velocities_and_angles()
                        print("[GameController] Switched to Walk mode (RB released)")

        return True
        
    def get_data(self):
        """
        Returns a dictionary with the current state of the controller.
        """
        self.process_events()  # Ensure we process events to update state

        output = {
            'stand_cmd': self.commands.stand,
            'sit_cmd': self.commands.sit,
        }
        if self.mode == 'walk':
            output.update({
                'v_x': self.v_x,
                'v_y': self.v_y,
                'v_rot': self.v_rot,
                'pitch': self.pitch,  # Pitch controlled by triggers in walk mode
                'height': self.height  # Height controlled by D-pad in walk mode
            })
        elif self.mode == 'stand':
            output.update({
                'yaw': self.yaw,
                'pitch': self.pitch,
                'height': self.height
            })
        
        return output

    def close(self):
        pygame.quit()


if __name__ == "__main__":
    try:
        controller = GameController()
        while True:
            data = controller.get_data()  # Continuously get data
            
            # Format float values to 2 decimal places
            formatted_data = {}
            for key, value in data.items():
                if isinstance(value, float):
                    formatted_data[key] = round(value, 2)
                else:
                    formatted_data[key] = value
            
            print(f"\r{' ' * 150}\r{formatted_data}", end="", flush=True)
            pygame.time.delay(50)  # Delay to avoid high CPU usage
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.close()