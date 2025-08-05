# This python file analyzes the control data for the set point controller.
# The file name is "Data" and it is located in the "ControlData" folder.

# First column is e_u for boat 1,
# Second column is e_u for boat 2,
# Third column is e_theta for boat 1,
# Fourth column is e_theta for boat 2,
# Get the time step from the params.json file under "simulation" -> "time_step".

import numpy as np
import pandas as pd
import json
import matplotlib.pyplot as plt
def load_params():
    with open('params.json', 'r') as f:
        params = json.load(f)
    return params

def analyze_control_data(file_path):
    segments = []
    current_segment = []
    
    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('# New setpoint'):
                # Save current segment if it has data
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
            elif line and not line.startswith('#'):
                # Parse data line
                try:
                    values = [float(x) for x in line.split()]
                    if len(values) == 4:
                        current_segment.append(values)
                except ValueError:
                    continue
    
    # Don't forget the last segment
    if current_segment:
        segments.append(current_segment)
    
    # Convert segments to DataFrames and extract columns
    segment_data = []
    for i, segment in enumerate(segments):
        if not segment:  # Skip empty segments
            continue
            
        df = pd.DataFrame(segment, columns=['e_u1', 'e_u2', 'e_theta1', 'e_theta2'])
        
        # Convert theta errors to degrees
        e_u1 = df['e_u1']
        e_u2 = df['e_u2']
        e_theta1 = df['e_theta1'] * 180 / np.pi
        e_theta2 = df['e_theta2'] * 180 / np.pi
        
        segment_data.append({
            'segment': i + 1,
            'e_u1': e_u1,
            'e_u2': e_u2,
            'e_theta1': e_theta1,
            'e_theta2': e_theta2
        })
    
    return segment_data

# main
if __name__ == "__main__":
    params = load_params()
    ts = params['simulation']['time_step'] 
    file_path = 'ControlData/Data.txt' 
    segment_data = analyze_control_data(file_path)

    tolerance_d = params['SetPointController']['tolerance_d']

    num_segments = len(segment_data)

    # Create separate figure for each segment
    for i, data in enumerate(segment_data):
        seg_num = data['segment']
        
        # Create time array for this segment
        time = np.arange(len(data['e_u1'])) * ts
        
        # Create 3 subplots for this segment
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
        fig.suptitle(f'Segment {seg_num} / {num_segments} - Control Errors', fontsize=16)
        
        # Plot 1: e_u1 and e_u2 together
        ax1.plot(time, data['e_u1'], 'b-', label='e_u1 (Boat 1)', linewidth=2)
        ax1.plot(time, data['e_u2'], 'r-', label='e_u2 (Boat 2)', linewidth=2)
        ax1.set_xlim([0, time[-1]])
        ax1.axhline(y=tolerance_d, color='green', linestyle='--', label='tolerance_d')
        ax1.axhline(y=-tolerance_d, color='green', linestyle='--')
        ax1.set_title('e_u for Both Boats [m]')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('e_u [m]')
        ax1.grid(True)
        ax1.legend()
        
        # Plot 2: e_theta1 only
        ax2.plot(time, data['e_theta1'], 'b-', label='e_theta1 (Boat 1)', linewidth=2)
        ax2.set_xlim([0, time[-1]])
        ax2.set_ylim([-20, 20])  # Limit y-axis to -20 to 20 degrees
        ax2.set_title('e_theta for Boat 1 [degrees]')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('e_theta1 [degrees]')
        ax2.grid(True)
        ax2.legend()
        
        # Plot 3: e_theta2 only
        ax3.plot(time, data['e_theta2'], 'r-', label='e_theta2 (Boat 2)', linewidth=2)
        ax3.set_xlim([0, time[-1]])
        ax3.set_ylim([-20, 20])  # Limit y-axis to -20 to 20 degrees
        ax3.set_title('e_theta for Boat 2 [degrees]')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('e_theta2 [degrees]')
        ax3.grid(True)
        ax3.legend()
        
        plt.tight_layout()
        plt.show()

    # # Print statistics for each segment
    # print(f"\nFound {len(segment_data)} segments:")
    # for data in segment_data:
    #     seg_num = data['segment']
    #     duration = len(data['e_u1']) * ts
    #     print(f"\nSegment {seg_num} (Duration: {duration:.2f}s, {len(data['e_u1'])} points):")
    #     print(f"  e_u1: min={data['e_u1'].min():.4f}, max={data['e_u1'].max():.4f}, final={data['e_u1'].iloc[-1]:.4f}")
    #     print(f"  e_u2: min={data['e_u2'].min():.4f}, max={data['e_u2'].max():.4f}, final={data['e_u2'].iloc[-1]:.4f}")  
    #     print(f"  e_theta1: min={data['e_theta1'].min():.2f}°, max={data['e_theta1'].max():.2f}°, final={data['e_theta1'].iloc[-1]:.2f}°")
    #     print(f"  e_theta2: min={data['e_theta2'].min():.2f}°, max={data['e_theta2'].max():.2f}°, final={data['e_theta2'].iloc[-1]:.2f}°")



