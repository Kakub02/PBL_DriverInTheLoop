def speed_to_pwm(speed_kmph, max_motor_speed_kmph, pwm_range):
    if speed_kmph > max_motor_speed_kmph:
        speed_kmph = actual_speed_kmph  #do dokonczenia max motor speed musi byc stale nie wiem moze ustawic na max rmp
#ale max speed jest stałe tylko jest w motor_listener_node --wojt
    pwm = (speed_kmph / max_motor_speed_kmph) * pwm_range
    return int(pwm)

def speed_to_rpm(speed_kmph, max_motor_speed_rpm):
    r = 0.0325
    if speed_kmph > 80:
        speed_kmph = 80
    
    RPM_SetPoint = (25/(3*3.14*r))*speed_kmph
    return RPM_SetPoint