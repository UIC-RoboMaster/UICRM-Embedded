# coding=utf-8

import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# 读取CSV数据
def load_data(file_path):
    data = pd.read_csv(file_path)
    return data

# 定义拟合函数
def current_model(X, k1, k2, k3, k4):
    angular_velocity, raw_current = X
    return raw_current * angular_velocity * k1 + abs(angular_velocity) ** 2 * k2 + abs(raw_current) ** 2 * k3 + k4

# 主函数
def main():
    # 读取CSV文件
    file_path = "data_record.csv"
    data = load_data(file_path)
    
    # 提取数据
    angular_velocity = data['angular_velocity'].values
    raw_current = data['raw_current'].values
    read_current = data['read_current'].values
    timestamp = data['timestamp'].values
    
    # 准备拟合数据
    X = np.vstack((angular_velocity, raw_current))
    
    # 使用最小二乘法拟合参数
    initial_guess = [0.001, 0.001, 0.001, 100]  # 初始参数猜测
    params, covariance = curve_fit(current_model, X, read_current, p0=initial_guess)
    
    # 获取拟合参数
    k1, k2, k3, k4 = params
    print(f"拟合参数: k1={k1:.6f}, k2={k2:.6f}, k3={k3:.6f}, k4={k4:.6f}")
    
    # 计算预测电流
    predicted_current = current_model(X, k1, k2, k3, k4)
    
    # 计算均方根误差（RMSE）
    rmse = np.sqrt(np.mean((predicted_current - read_current)**2))
    print(f"均方根误差(RMSE): {rmse:.2f}")

    # 保存预测电流
    data['predicted_current'] = predicted_current
    data.to_csv('predicted_current.csv', index=False)
    
    # 绘制预测电流与实际电流的对比图
    plt.figure(figsize=(12, 6))
    
    # 使用数据点编号作为x轴
    data_points = np.arange(len(read_current))
    
    # 绘制实际电流
    plt.plot(data_points, read_current, 'b-', label='Read Current')
    # 绘制预测电流
    plt.plot(data_points, predicted_current, 'r--', label='Predicted Current')
    
    plt.xlabel('Data Point Index')
    plt.ylabel('Current')
    plt.legend()
    plt.grid(True)
    
    # 保存图像
    plt.savefig('current_comparison.png')
    plt.show()
    
    # 返回参数
    return params

if __name__ == "__main__":
    main()
