import argparse
from gui import *
from data_manager import DataManager
from config import USING_STEREO_CAMERA

def main(broker_address):
    # --- Start the Data Manager
    data_manager = DataManager(broker_address)
    data_manager.start()

    # --- Build the GUI application
    app = ModularGUI(data_manager)
    stream_address = f"rtsp://{broker_address}:8554/spot-stream"

    # --- Add all blocks to the application window following the predefined grid of 16x9
    app.add_block(SwappableBlock(
        app, 
        blocks=[
            MetaQuestDataBlock(app), 
            SpotInputsDataBlock(app),  
            SpotStatusDataBlock(app)
        ],
        swap_key="Space",  # Press Space to swap
        name="Visualization Blocks"
        ), 
        grid_x=0, grid_y=0
    )

    app.add_block(VideoDisplayBlock(app, stream_address, USING_STEREO_CAMERA), grid_x=2, grid_y=0)
    app.add_block(KatvrDataBlock(app), grid_x=0, grid_y=6)
    app.add_block(JoystickV1Block(app, key_config="wasd", name="Left Joystick"), grid_x=2, grid_y=6)
    app.add_block(JoystickV1Block(app, key_config="arrows", name="Right Joystick"), grid_x=4, grid_y=6)
    app.add_block(JoystickV2Block(app), grid_x=6, grid_y=6)
    app.add_block(ControlSourceBlock(app), grid_x=8, grid_y=8)
    app.add_block(CommandControlBlock(app), grid_x=8, grid_y=6)
    app.add_block(PIDGraphBlock(app), grid_x=11, grid_y=6)
    
    # --- Start the GUI main loop
    app.mainloop()  # This blocks until the GUI window is closed

    print("\n[MAIN] Stopping all processes...")
    data_manager.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Send control data to the MQTT server")
    parser.add_argument(
        'ip_address',
        type=str,
        nargs='?',
        default='192.168.20.3',  
        help='The IP address of the MQTT server'
    )
    args = parser.parse_args()
    print(f"[MAIN] Starting application with broker address {args.ip_address}")
    main(broker_address=args.ip_address)
