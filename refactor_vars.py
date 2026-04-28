#!/usr/bin/env python3
"""Refactor variable names in main.cpp to use camelCase conventions."""

import re
import sys

def refactor_variables(file_path):
    """Apply all variable name replacements."""
    
    replacements = [
        ('display_speed1', 'DISPLAY_SPEED_FAST'),
        ('display_speed2', 'DISPLAY_SPEED_MEDIUM'),
        ('display_speed3', 'DISPLAY_SPEED_SLOW'),
        ('newSettingsAvailiable', 'newSettingsAvailable'),
        ('getUserRecieved', 'getUserReceived'),
        ('getRobotRecieved', 'getRobotReceived'),
        ('getJudgeRecieved', 'getJudgeReceived'),
        ('retryJudgeRecieved', 'retryJudgeReceived'),
        ('retryScoreRecieved', 'retryScoreReceived'),
        ('timeDiffrence', 'timeDifference'),
        ('NRFInterruptTime', 'nrfInterruptTime'),
        ('NRFInterruptFlag', 'nrfInterruptFlag'),
        ('synchroCounter', 'syncCounter'),
        ('synchroSuccess', 'syncSuccess'),
        ('synchroTime', 'syncTime'),
        ('finalFormatedTime', 'finalFormattedTime'),
        ('counter_m', 'counterMinutes'),
        ('counter_s', 'counterSeconds'),
        ('counter_ms', 'counterMilliseconds'),
        ('RC_522_SS', 'RC522_SS'),
        ('RC_522_RST', 'RC522_RST'),
        ('synchronize_time_receiver', 'synchronizeTimeReceiver'),
        ('synchronize_time_transmitter', 'synchronizeTimeTransmitter'),
        ('last_received_interrupt_time', 'lastReceivedInterruptTime'),
        ('last_click_time', 'lastClickTime'),
        ('repeat_qr', 'repeatQRTimeout'),
        ('repeat_rfid', 'repeatRFIDTimeout'),
        ('ir_brake_active', 'irBrakeActive'),
        ('time_brake_sensor', 'timeBrakeSensor'),
        ('startAddress', 'startNRFAddress'),
        ('finishAddress', 'finishNRFAddress')
    ]
    
    # Read the file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Apply replacements with word boundaries
    for old_name, new_name in replacements:
        pattern = r'\b' + re.escape(old_name) + r'\b'
        content = re.sub(pattern, new_name, content)
        print(f'✓ Replaced "{old_name}" with "{new_name}"')
    
    # Write the modified content back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print('\n✓ All replacements completed successfully!')
    return True

if __name__ == '__main__':
    file_path = r'D:\Programowanie\INZYNIERKA\LF_gate_project\src\main.cpp'
    try:
        refactor_variables(file_path)
        sys.exit(0)
    except Exception as e:
        print(f'✗ Error: {e}', file=sys.stderr)
        sys.exit(1)
