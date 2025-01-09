# 允许使用osimpipeline git子模块。
import sys
# 将'code'目录添加到系统路径中，以便可以导入该目录下的模块。
sys.path.insert(1, 'code')
# 将'osimpipeline'目录添加到系统路径中，以便可以导入该目录下的模块。
sys.path.insert(1, 'osimpipeline')
# 将'osimpipeline/osimpipeline'目录添加到系统路径中，以便可以导入该目录下的模块。
# 该行代码将'osimpipeline/osimpipeline'目录添加到系统路径中，以便可以导入该目录下的模块。
sys.path.insert(1, 'osimpipeline/osimpipeline')

import os
import yaml
import numpy as np
# 打开'config.yaml'文件，并使用yaml.safe_load()函数加载文件内容。
with open('config.yaml') as f:
    config = yaml.safe_load(f)
# 检查'config.yaml'文件中是否包含'opensim_home'字段。
if 'opensim_home' not in config:
    # 如果'opensim_home'字段不存在，则抛出异常。
    raise Exception('You must define the field `opensim_home` in config.yaml '
                    'to point to the root of your OpenSim 4.0 (or later) '
                    'installation.')
# 将'opensim_home'字段指向的OpenSim安装目录添加到系统路径中。
# 将OpenSim的Python SDK路径添加到系统路径中，以便可以导入OpenSim的Python模块。
# 这里的config['opensim_home']是指向OpenSim安装目录的路径。
# os.path.join()函数用于将该路径与'sdk'和'python'目录连接起来，形成完整的SDK路径。
sys.path.insert(1, os.path.join(config['opensim_home'], 'sdk', 'python'))

# 设置doit的配置，verbosity设置为2，表示输出详细信息。
# default_tasks设置为None，表示不指定默认任务。
DOIT_CONFIG = {
        'verbosity': 2,
        'default_tasks': None,
        }

# 导入matplotlib库，并设置字体、错误条样式、线条样式和图例字体大小。
# Settings for plots.
import matplotlib.pyplot as plt
plt.rc('font', family='Helvetica, Arial, sans-serif', size=8)
plt.rc('errorbar', capsize=1.5)
plt.rc('lines', markeredgewidth=1)
plt.rc('legend', fontsize=8)

# 导入osimpipeline库，并设置别名为osp。
import osimpipeline as osp
# 从osimpipeline库中导入postprocessing模块，并设置别名为pp。
from osimpipeline import postprocessing as pp

# 导入vital_tasks模块，该抹模块用于注册任务。
# This line is necessary for registering the tasks with python-doit.
from vital_tasks import *

# 导入tasks模块，该模块用于定义任务。
# Custom tasks for this project.
from tasks import *

# 导入helpers模块，该模块用于定义辅助函数。
# Custom helper functions for this project
from helpers import *

# 定义模型文件名
model_fname = 'Rajagopal2015_passiveCal_hipAbdMoved_EBCForces_ankleBushings_toesAligned.osim'
# 定义模型文件路径
generic_model_fpath = os.path.join('model', model_fname)
# 创建Study对象，指定研究名称和模型文件的路径。
study = osp.Study('ankle_perturb_sim', 
    generic_model_fpath=generic_model_fpath)

# 设置研究对象的步行速度为1.25m/s
# Set the treadmill walking speed for the study
study.walking_speed = 1.25

# 添加通用任务
# Generic model file
# ------------------
# 添加任务，复制模型文件到结果目录
study.add_task(TaskCopyGenericModelFilesToResults)
# 添加任务，将标记集应用到通用模型
study.add_task(TaskApplyMarkerSetToGenericModel)

# 定义用于计算误差的模型标记
# Model markers to compute errors for
marker_suffix = ['ASI', 'PSI', 'TH1', 'TH2', 'TH3', 'CAL', 'TOE', 'MT5']
error_markers = ['*' + marker for marker in marker_suffix] 
error_markers.append('CLAV')
error_markers.append('C7')
study.error_markers = error_markers

# 定义权重
# Define weights
scale = 1.0
# 这些权重的分别是指
study.weights = {
    'state_tracking_weight':    50 * scale,  # 状态跟踪权重
    'control_weight':           25 * scale,   # 控制权重
    'grf_tracking_weight':      7500 * scale, # 地面反作用力跟踪权重
    'torso_orientation_weight': 10 * scale,   # 躯干方向权重
    'feet_orientation_weight':  10 * scale,   # 足部方向权重
    'control_tracking_weight':  0 * scale,    # 控制跟踪权重
    'aux_deriv_weight':         1000 * scale, # 辅助导数权重
    'acceleration_weight':      1 * scale,    # 加速度权重
}
# 设置约束容差和收敛容差
study.constraint_tolerance = 1e-4
study.convergence_tolerance = 1e-2

# 设置最大扰动扭矩
# Maximum perturbation torque
# ???这个地方的参数含义需要根据后面代码确认
study.torques = [0, 10]
# 设置时间步长
study.times = [20, 25, 30, 35, 40, 45, 50, 55, 60] 
# 设置上升时间
study.rise = 10
# 设置下降时间
study.fall = 5
# 设置跖屈峰值扭矩
study.subtalar_peak_torques = [-10, 0, 10]
# 设置跖屈后缀
study.subtalar_suffixes = list()
# 遍历跖屈峰值扭矩，设置study名字后缀为_subtalar-10和_subtalar10
for peak_torque in study.subtalar_peak_torques:
    if peak_torque:
        study.subtalar_suffixes.append(f'_subtalar{peak_torque}')
    else:
        study.subtalar_suffixes.append('')
# 设置腰椎刚度
study.lumbar_stiffnesses = [0.1, 1.0, 10.0]
# 设置颜色映射
colormap = 'plasma'
cmap = plt.get_cmap(colormap)
indices = np.linspace(0, 1.0, len(study.subtalar_suffixes)) 
study.subtalar_colors = [cmap(idx) for idx in indices]

# Add subject tasks
# -----------------
# 导入subject对应的py文件模块，并将其添加到研究对象中。
import subject01
subject01.add_to_study(study)

import subject02
subject02.add_to_study(study)

import subject04
subject04.add_to_study(study)

import subject18
subject18.add_to_study(study)

import subject19
subject19.add_to_study(study)

# ???添加任务，复制运动捕捉数据，具体数据需要看看
# Copy mocap data
# ---------------
study.add_task(TaskCopyMotionCaptureData, walk125=(2, ''))

# 设置绘制设置
# Plot settings
# -------------
# 设置subjects列表，包含subject01、subject02、subject04、subject18和subject19
subjects = [
            'subject01', 
            'subject02', 
            'subject04', 
            'subject18', 
            'subject19'
            ]
# 设置masses列表，包含每个subject的质量
masses = [
          study.get_subject(1).mass,
          study.get_subject(2).mass,
          study.get_subject(4).mass,
          study.get_subject(18).mass,
          study.get_subject(19).mass
          ]
# 设置plot_torques列表，包含每个subject的扰动扭矩
study.plot_torques = [0, 10, 10, 10, 0]

# 设置plot_subtalars列表，包含每个subject的跖屈后缀
plot_subtalars = list()
# 添加的跖屈后缀为：_subtalar-10、_subtalar0、_subtalar10
plot_subtalars.append(study.subtalar_suffixes[0]) 
plot_subtalars.extend(study.subtalar_suffixes)
plot_subtalars.append(study.subtalar_suffixes[-1])
study.plot_subtalars = plot_subtalars

# 设置plot_colors列表，包含每个subject的颜色
# 为每个subject设置颜色
# 定义颜色变量，使用RGB值并将其归一化到0到1之间
lightorange = [c / 255.0 for c in [253, 141, 60]]  # 浅橙色
orange =      [c / 255.0 for c in [217, 71, 1]]    # 橙色
blue =        [c / 255.0 for c in [33, 113, 181]]   # 蓝色
lightblue =   [c / 255.0 for c in [107, 174, 214]]  # 浅蓝色

# 将颜色应用到study.plot_colors中
# 这里我们为每个subject指定不同的颜色
study.plot_colors = [
    pp.adjust_lightness(lightorange, amount=1.0),  # subject01的颜色
    pp.adjust_lightness(orange, amount=1.0),        # subject02的颜色
    'black',                                        # subject04的颜色
    pp.adjust_lightness(blue, amount=1.0),         # subject18的颜色
    pp.adjust_lightness(lightblue, amount=1.0)     # subject19的颜色
]
# 添加绘图任务
# Methods figure
# --------------
study.add_task(TaskPlotMethodsFigure, subjects, study.times)

# 添加验证任务
# Validate
# --------
study.add_task(TaskPlotUnperturbedResults, subjects, masses, study.times)
study.add_task(TaskValidateTrackingErrors, subjects, masses, study.times)
study.add_task(TaskValidateMarkerErrors)
study.add_task(TaskComputeCenterOfMassTimesteppingError, subjects, study.times)
study.add_task(TaskValidateAccelerationsVersusGRFs, subjects, study.times)
study.add_task(TaskValidateAccelerationsVersusVelocitiess, subjects, study.times)
study.add_task(TaskValidateMuscleActivity, subjects)
study.add_task(TaskComputeObjectiveContributions, subjects)

# 添加统计任务
# Statistics
# ----------
study.add_task(TaskCreateCenterOfMassStatisticsTables, subjects, study.times)
study.add_task(TaskCreateCenterOfPressureStatisticsTables, subjects, study.times)
study.add_task(TaskCreateWholeBodyAngularMomentumStatisticsTables, subjects, study.times)
study.add_task(TaskRunStatistics, study.times)
study.add_task(TaskAggregateCenterOfMassStatistics, study.times)
study.add_task(TaskAggregateCenterOfPressureStatistics, study.times)
study.add_task(TaskAggregateWholeBodyAngularMomentumStatistics, study.times)

# 添加CoM质心分析任务
# Center-of-mass analysis
# -----------------------
study.add_task(TaskPlotCenterOfMassVector, subjects, study.times)
study.add_task(TaskPlotInstantaneousCenterOfMass, subjects, study.times)
study.add_task(TaskPlotCOMVersusCOP, subjects, study.times)
study.add_task(TaskPlotCOMVersusPeakTorque, subjects)

# 添加CoP压力中心分析任务
# Center-of-pressure analysis
# ---------------------------
study.add_task(TaskPlotCenterOfPressureVector, subjects, study.times)
study.add_task(TaskPlotInstantaneousCenterOfPressure, subjects, study.times)

# 添加全身角动量分析任务
# Whole-body angular momentum analysis
# ------------------------------------
study.add_task(TaskPlotInstantaneousWholeBodyAngularMomentum, subjects, study.times)

# 添加扰动功率分析任务
# Device powers
# -------------
study.add_task(TaskCreatePerturbationPowersTable, subjects)
study.add_task(TaskCreatePerturbationPowersTable, subjects,
    torque_actuators=True)
study.add_task(TaskPlotPerturbationPowers, subjects)