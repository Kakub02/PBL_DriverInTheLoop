from mpu6050 import mpu6050

mpu = mpu6050(0x68)

def get_accel():
    accel_data = mpu.get_accel_data()
    X_mpu = accel_data['x']
    Y_mpu = accel_data['y']
    
    return X_mpu, Y_mpu