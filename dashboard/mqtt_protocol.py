"""Pico binary-v2 wire format. No dependency on the firmware or MQTT client."""
STATES = ('IDLE', 'FOLLOW_LINE', 'TURNING', 'AVOIDING', 'RECOVER_LINE', 'FAULT', 'RETRY_BARCODE')
MOTOR_MODES = ('IDLE', 'VELOCITY', 'DISTANCE', 'TURN')
MOTOR_FAULTS = ((1, 'Encoder counts'), (2, 'Control timing'), (4, 'Wheel stalled'), (8, 'Command timeout'))
VEHICLE_FAULTS = ((1, 'Motor'), (2, 'Encoder'), (4, 'IMU'), (8, 'Ultrasonic echo'), (16, 'Wi-Fi'))
TELEMETRY_TOPICS = ('vehicle', 'motion', 'line', 'terrain', 'obstacle', 'controller', 'sonar', 'barcode_map')
BARCODE_ACTIONS = ('left', 'right', 'straight', 'uturn')
START_BLOCK_REPOSITION = 8
STOP_REASON_OPERATOR = 1
STOP_REASONS = (
    None, 'Operator STOP received.', 'Line follower requested a stop.',
    'Pico lost valid local IR samples.', 'Motor control fault.',
    'Command connection lost.', 'Obstacle avoidance stopped the car.',
    'The barcode turn could not confirm the branch tape.',
)
AVOIDANCE_PHASES = (
    'idle', 'scanning ahead', 'turning off the tape', 'leaving the tape', 'aligning for the bypass',
    'passing the obstacle', 'turning toward the tape', 'searching for the tape', 'centring on the tape',
    'realigning with the tape', 'aligning to pass farther', 'complete', 'stopped',
    'confirming the tape', 'backing up', 'waiting for a route', 'checking side clearance',
)
AVOIDANCE_FAILURES = (
    None, 'Obstacle scan timed out', 'Obstacle could not be confirmed', 'Width could not be confirmed',
    'Too little room to turn', 'Side clearance too small', 'No usable sonar echo',
    'Obstacle too close or clearance travel exhausted', 'Motor maneuver failed', 'Tape could not be reacquired',
    'Operator stopped the detour', 'Controller interlock stopped the detour', 'Maneuver timed out',
    'Left edge outside scan limits', 'Right edge outside scan limits', 'Both edges outside scan limits',
)
TERRAIN_PHASES = ('Flat', 'Ascending', 'Descending', 'Unavailable')

def command_payload(command):
    if command == 'start': return b'\x01'
    if command == 'stop': return b'\x00'
    if command == 'reset': return b'\x02'
    raise ValueError('command must be start, stop or reset')

def barcode_map_payload(mapping):
    if not isinstance(mapping, dict) or set(mapping) != set(BARCODE_ACTIONS):
        raise ValueError('Choose a letter for each of the four actions.')
    word = 0xA << 20
    seen = set()
    for index, action in enumerate(BARCODE_ACTIONS):
        letter = mapping[action]
        if not isinstance(letter, str) or len(letter) != 1 or not 'A' <= letter <= 'Z':
            raise ValueError('Choose letters from A to Z.')
        if letter in seen:
            raise ValueError('Use a different letter for each action.')
        seen.add(letter)
        word |= (ord(letter) - ord('A') + 1) << (5 * index)
    return word.to_bytes(3, 'little')

def decode_telemetry(kind, payload):
    sizes = dict(vehicle=(1,), motion=(9,), line=(1,), terrain=(2, 3),
                 obstacle=(6,), controller=(25, 31), sonar=(4,), barcode_map=(3,))
    if len(payload) not in sizes.get(kind, ()):
        raise ValueError('Incorrect binary telemetry length')
    word = int.from_bytes(payload, 'little')
    def bits(offset, width): return (word >> offset) & ((1 << width) - 1)
    if kind == 'barcode_map':
        letters = [bits(5 * index, 5) for index in range(len(BARCODE_ACTIONS))]
        if bits(20, 4) != 0xA or min(letters) < 1 or max(letters) > 26 or len(set(letters)) != 4:
            raise ValueError('Invalid barcode mapping')
        return dict(zip(BARCODE_ACTIONS, (chr(ord('A') + letter - 1) for letter in letters)))
    if kind == 'controller':
        if (payload[0],len(payload)) not in ((1,25),(2,31)) or payload[1] >= len(STATES) or payload[2] >= len(STOP_REASONS) or payload[3] > 8 or payload[24] > 8 or (payload[0]==1 and payload[4] & 128) or payload[5] > 15:
            raise ValueError('Invalid controller diagnostics')
        raw = [int.from_bytes(payload[i:i+2], 'little') for i in (6,8,10)]
        if max(raw)>1000 or max(payload[12:14])>126 or any(x%2 for x in payload[12:14]):
            raise ValueError('Invalid controller sensor or target range')
        blocks = ('Ready for START', 'Motor startup not ready', 'Wi-Fi/MQTT not ready',
                  'Motor fault latched', 'STOP / reset pending', 'No valid IR readings',
                  'IR readings are stale', 'Junction / line stop latched', 'Car must be put back on the tape')
        hints = ('Controller start checks pass.', 'Wait at least five seconds after powering the Pico.',
                 'Wait for Wi-Fi, broker and command subscription.',
                 'Inspect motor faults before resetting or restarting.',
                 'Allow the controller to process STOP before retrying.',
                 'Check the IR sensor AO connections and ADC read errors on USB serial.',
                 'The sensor task is not supplying fresh data; inspect USB serial.',
                 'Place the car on a straight away from crossings, then restart the Pico. The junction test remains halted until restart.',
                 'Place the car on a straight section of tape before retrying START.')
        age = int.from_bytes(payload[14:16], 'little')
        return dict(state=STATES[payload[1]], stop_reason=payload[2], stop_reason_label=STOP_REASONS[payload[2]],
                    start_block=payload[3],
                    start_block_label=blocks[payload[3]], start_hint=hints[payload[3]],
                    motor_ready=bool(payload[4]&1), line_valid=bool(payload[4]&2),
                    line_stop=bool(payload[4]&4), obstacle_blocked=bool(payload[4]&8),
                    start_pending=bool(payload[4]&16), stop_pending=bool(payload[4]&32),
                    link_ready=bool(payload[4]&64), motor_faults=payload[5],
                    left_dark=raw[0], centre_dark=raw[1], right_dark=raw[2],
                    left_target_rpm=payload[12], right_target_rpm=payload[13],
                    line_age_ms=None if age==65535 else age,
                    uptime_ms=int.from_bytes(payload[16:20],'little'),
                    telemetry_drops=int.from_bytes(payload[20:24],'little'),
                    last_start_reject=payload[24], last_reject_label=blocks[payload[24]],
                    last_reject_hint=hints[payload[24]], reset_supported=payload[0]>=2,
                    reset_pending=bool(payload[4]&128),
                    timing_gap_us=int.from_bytes(payload[25:29],'little') if payload[0]>=2 else None,
                    timing_limit_us=int.from_bytes(payload[29:31],'little') if payload[0]>=2 else None)
    if kind == 'vehicle':
        state, faults = bits(0, 3), bits(3, 5)
        if state >= len(STATES): raise ValueError('Unknown vehicle state')
        return dict(state=STATES[state], state_code=state, faults=faults,
                    fault_labels=[label for bit, label in VEHICLE_FAULTS if faults & bit])
    if kind == 'motion':
        fields = dict(mode=bits(0, 2), speed_level=bits(2, 3), faults=bits(5, 4),
                      left_rpm=bits(9, 9), right_rpm=bits(18, 9),
                      left_pwm=bits(27, 7), right_pwm=bits(34, 7),
                      left_progress=bits(41, 13), right_progress=bits(54, 13))
        if word >> 67 or fields['speed_level'] > 4 or max(fields['left_rpm'], fields['right_rpm']) > 300 or max(fields['left_pwm'], fields['right_pwm']) > 100:
            raise ValueError('Invalid motion range or reserved bits')
        fields['mode_name'] = MOTOR_MODES[fields['mode']]
        fields['fault_labels'] = [label for bit, label in MOTOR_FAULTS if fields['faults'] & bit]
        return fields
    if kind == 'line':
        ir, barcode = bits(0, 3), bits(3, 5)
        if barcode > 26: raise ValueError('Invalid line payload')
        return dict(ir_bits=ir, left_black=bool(ir & 1), centre_black=bool(ir & 2),
                    right_black=bool(ir & 4), barcode_code=barcode,
                    barcode='None' if barcode == 0 else chr(ord('A') + barcode - 1))
    if kind == 'terrain':
        height = bits(0, 11)
        max_height = None
        if len(payload) == 2:
            phase = bits(11, 3)
            if word >> 14 or phase > 5:
                raise ValueError('Invalid legacy terrain payload')
        else:
            phase = bits(11, 2)
            peak = bits(13, 11)
            if peak != 2047:
                if not height <= peak <= 2000:
                    raise ValueError('Invalid maximum hump height')
                max_height = peak
        if height > 2000:
            raise ValueError('Invalid hump height')
        return dict(height_mm=height, max_height_mm=max_height, phase=phase,
                    phase_label=TERRAIN_PHASES[min(phase, 3)])
    if kind == 'sonar':
        forward, phase, failed_phase, failure = bits(0, 12), bits(12, 5), bits(17, 5), bits(22, 4)
        if (word >> 26 or (4000 < forward < 4094) or max(phase, failed_phase) >= len(AVOIDANCE_PHASES)):
            raise ValueError('Invalid sonar diagnostics')
        return dict(forward_mm='Open' if forward == 4094 else None if forward == 4095 else forward,
                    phase=AVOIDANCE_PHASES[phase], failed_phase=AVOIDANCE_PHASES[failed_phase],
                    failure=AVOIDANCE_FAILURES[failure])
    fields = dict(nearest_mm=bits(0, 12), width_mm=bits(12, 11),
                  left_mm=bits(23, 12), right_mm=bits(35, 12))
    if (word >> 47 or fields['nearest_mm'] > 4000 or
            (fields['width_mm'] > 2000 and fields['width_mm'] != 2047)):
        raise ValueError('Invalid obstacle payload')
    if fields['width_mm'] == 2047:
        fields['width_mm'] = None
    for side in ('left_mm', 'right_mm'):
        if fields[side] in (4094, 4095):
            fields[side] = 'Open' if fields[side] == 4094 else None
        elif fields[side] > 4000:
            raise ValueError('Invalid obstacle payload')
    return fields
