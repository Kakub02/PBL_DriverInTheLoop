from mpu6050 import mpu6050

mpu = mpu6050(0x68)

def get_gyro():
    gyro_data = mpu.get_gyro_data()
    X_mpu = gyro_data['x']
    Y_mpu = gyro_data['y']
    
    return X_mpu, Y_mpu