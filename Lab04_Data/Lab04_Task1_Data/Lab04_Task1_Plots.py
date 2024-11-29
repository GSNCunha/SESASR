import pandas as pd
import matplotlib as plt
import numpy as np

def calculate_rmse_mae(gt, est):
    rmse = np.sqrt(np.mean((gt - est)**2, axis=0))
    mae = np.mean(np.abs(gt - est), axis=0)
    return rmse, mae

def plot_data(df):
    # Filter data by source
    df_gt = df[df['source'] == '/ground_truth']
    df_ekf = df[df['source'] == '/ekf']
    df_odom = df[df['source'] == '/odom']

    # Plot positions
    plt.figure(figsize=(12, 8))
    plt.plot(df_gt['x'], df_gt['y'], label='Ground Truth')
    plt.plot(df_ekf['x'], df_ekf['y'], label='EKF')
    plt.plot(df_odom['x'], df_odom['y'], label='Odometry')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Position Comparison')
    plt.grid(True)
    plt.show()

    # Plot yaw
    plt.figure()
    plt.plot(df_gt['yaw'], label='Ground Truth')
    plt.plot(df_ekf['yaw'], label='EKF')
    plt.plot(df_odom['yaw'], label='Odometry')
    plt.xlabel('Time')
    plt.ylabel('Yaw')
    plt.legend()
    plt.title('Yaw Comparison')
    plt.grid(True)
    plt.show()

    # Calculate and plot RMSE and MAE
    gt = df_gt[['x', 'y', 'yaw']].values
    ekf = df_ekf[['x', 'y', 'yaw']].values
    odom = df_odom[['x', 'y', 'yaw']].values

    ekf_rmse, ekf_mae = calculate_rmse_mae(gt, ekf)
    odom_rmse, odom_mae = calculate_rmse_mae(gt, odom)

    plt.figure()
    plt.bar(['X', 'Y', 'Yaw'], ekf_rmse, label='EKF RMSE')
    plt.bar(['X', 'Y', 'Yaw'], odom_rmse, label='Odometry RMSE')
    plt.ylabel('RMSE')
    plt.legend()
    plt.title('RMSE Comparison')
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.bar(['X', 'Y', 'Yaw'], ekf_mae, label='EKF MAE')
    plt.bar(['X', 'Y', 'Yaw'], odom_mae, label='Odometry MAE')
    plt.ylabel('MAE')
    plt.legend()
    plt.title('MAE Comparison')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    df = pd.read_csv('Lab04_Task1.csv')
    print(df['/ekf/pose/pose/orientation/yaw'])
    plot_data(df)