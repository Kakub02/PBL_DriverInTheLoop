def speed_to_pwm(speed_kmph, max_motor_speed_kmph, pwm_range):
    if speed_kmph > max_motor_speed_kmph:
        speed_kmph = actual_speed_kmph  #do dokonczenia max motor speed musi byc stale nie wiem moze ustawic na max rmp
#ale max speed jest stałe tylko jest w motor_listener_node --wojt
    pwm = (speed_kmph / max_motor_speed_kmph) * pwm_range
    return int(pwm)